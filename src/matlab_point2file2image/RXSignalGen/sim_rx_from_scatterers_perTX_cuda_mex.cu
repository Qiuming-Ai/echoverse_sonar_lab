// sim_rx_from_scatterers_perTX_cuda_mex.cu
// -----------------------------------------------------------------------------
// CUDA加速版本的超声回波合成（Born 近似，单次散射），支持每个发射阵元不同激励
// 编译方法（选择其一）：
//   方法1（推荐）：mexcuda -R2018a NVCCFLAGS='-allow-unsupported-compiler' sim_rx_from_scatterers_perTX_cuda_mex.cu
//   方法2：setenv('NVCCFLAGS', '-allow-unsupported-compiler'); mexcuda -R2018a sim_rx_from_scatterers_perTX_cuda_mex.cu
// -----------------------------------------------------------------------------

#include "mex.h"

// 在包含 CUDA 头文件之前，尝试绕过 Visual Studio 版本检查
// 注意：这需要 nvcc 传递 -allow-unsupported-compiler 标志
// 如果环境变量方法不起作用，可能需要手动修改 host_config.h 文件

#include <cuda_runtime.h>
#include <cufft.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <limits>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CHECK_CUDA(stmt) do{ \
    cudaError_t err = (stmt); \
    if (err != cudaSuccess){ \
        mexErrMsgIdAndTxt("simrx:cuda", "CUDA error %d: %s", (int)err, cudaGetErrorString(err)); \
    } \
}while(0)

#define CHECK_CUFFT(stmt) do{ \
    cufftResult err = (stmt); \
    if (err != CUFFT_SUCCESS){ \
        mexErrMsgIdAndTxt("simrx:cufft", "cuFFT error %d", (int)err); \
    } \
}while(0)

#define CHECK_CUFFT(stmt) do{ \
    cufftResult err = (stmt); \
    if (err != CUFFT_SUCCESS){ \
        mexErrMsgIdAndTxt("simrx:cufft", "cuFFT error %d", (int)err); \
    } \
}while(0)

// 性能分析宏
#define PROFILE_START(name) \
    cudaEvent_t start_##name, stop_##name; \
    float time_##name = 0.0f; \
    cudaEventCreate(&start_##name); \
    cudaEventCreate(&stop_##name); \
    cudaEventRecord(start_##name);

#define PROFILE_STOP(name) \
    cudaEventRecord(stop_##name); \
    cudaEventSynchronize(stop_##name); \
    cudaEventElapsedTime(&time_##name, start_##name, stop_##name); \
    cudaEventDestroy(start_##name); \
    cudaEventDestroy(stop_##name);

#define PROFILE_PRINT(name, desc) \
    mexPrintf("[PROFILE] %s: %.3f ms\n", desc, time_##name);

// ==================== CUDA Kernels ====================

// Kernel 1: 计算所有时延并找到最小最大值（使用共享内存reduction）
template<typename Real>
__global__ void compute_delays_find_range(
    const Real* P,      // [K x 3]
    const Real* TX,     // [Nt x 3]
    const Real* RX,     // [Nr x 3]
    const Real* delayK, // [K]
    Real c,
    Real fs,
    int K, int Nt, int Nr,
    int round_mode,     // 0=round, 1=floor, 2=ceil
    Real* t_min_out,   // 输出：最小值
    Real* t_max_out)   // 输出：最大值
{
    extern __shared__ float sdata[];
    float* s_min = sdata;
    float* s_max = sdata + blockDim.x;

    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = K * Nr;

    Real t_min_local = 1e30f;
    Real t_max_local = -1e30f;

    if (idx < total) {
        int k = idx / Nr;
        int r = idx % Nr;

        Real Pk[3] = {P[k], P[k+K], P[k+2*K]};
        Real Rr[3] = {RX[r], RX[r+Nr], RX[r+2*Nr]};

        // 计算 Rrx
        Real dx = Pk[0] - Rr[0];
        Real dy = Pk[1] - Rr[1];
        Real dz = Pk[2] - Rr[2];
        Real Rrx = sqrtf(dx*dx + dy*dy + dz*dz);

        // 对每个TX计算时延
        for (int t = 0; t < Nt; t++) {
            Real TXt[3] = {TX[t], TX[t+Nt], TX[t+2*Nt]};
            dx = TXt[0] - Pk[0];
            dy = TXt[1] - Pk[1];
            dz = TXt[2] - Pk[2];
            Real Rtx = sqrtf(dx*dx + dy*dy + dz*dz);

            Real tau = (Rtx + Rrx) / c + delayK[k];
            
            t_min_local = fminf(t_min_local, tau);
            t_max_local = fmaxf(t_max_local, tau);
        }
    }

    s_min[tid] = t_min_local;
    s_max[tid] = t_max_local;
    __syncthreads();

    // Block-level reduction
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            s_min[tid] = fminf(s_min[tid], s_min[tid + s]);
            s_max[tid] = fmaxf(s_max[tid], s_max[tid + s]);
        }
        __syncthreads();
    }

    // 写入全局内存（使用原子操作）
    if (tid == 0) {
        // 使用循环CAS实现原子最小最大值更新
        unsigned int* t_min_ui = (unsigned int*)t_min_out;
        unsigned int* t_max_ui = (unsigned int*)t_max_out;
        
        unsigned int old_min_ui, new_min_ui, old_max_ui, new_max_ui;
        float old_min, new_min, old_max, new_max;
        
        // 原子更新最小值
        do {
            old_min_ui = *t_min_ui;
            old_min = __uint_as_float(old_min_ui);
            new_min = fminf(old_min, s_min[0]);
            new_min_ui = __float_as_uint(new_min);
        } while (atomicCAS(t_min_ui, old_min_ui, new_min_ui) != old_min_ui);
        
        // 原子更新最大值
        do {
            old_max_ui = *t_max_ui;
            old_max = __uint_as_float(old_max_ui);
            new_max = fmaxf(old_max, s_max[0]);
            new_max_ui = __float_as_uint(new_max);
        } while (atomicCAS(t_max_ui, old_max_ui, new_max_ui) != old_max_ui);
    }
}

// 辅助函数：取整
template<typename Real>
__device__ inline int round_idx(Real t, Real fs, int round_mode) {
    Real idx_f = t * fs;
    switch (round_mode) {
        case 0: return (int)roundf(idx_f);  // round
        case 1: return (int)floorf(idx_f);  // floor
        case 2: return (int)ceilf(idx_f);   // ceil
        default: return (int)roundf(idx_f);
    }
}

// 稀疏IR元素结构
struct SparseIRElem {
    int idx;      // IR索引
    float val;    // IR值
    int tx;       // TX索引
};

// Kernel 2: 构建IR（每个接收通道）- 返回稀疏表示
template<typename Real>
__global__ void build_ir_per_rx_sparse(
    const Real* __restrict__ P,      // [K x 3]
    const Real* __restrict__ A,      // [K]
    const Real* __restrict__ TX,     // [Nt x 3]
    const Real* __restrict__ RX,     // [Nr x 3]
    const Real* __restrict__ delayK, // [K]
    Real c,
    Real fs,
    Real t0_ref,
    int K, int Nt, int Nr, int r,
    int round_mode,
    int atten_mode,
    int Lir,
    SparseIRElem* sparse_ir,         // 输出：稀疏IR元素
    int* sparse_count)               // 输出：非零元素计数
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= K) return;

    int k = idx;
    Real Pk[3] = {P[k], P[k+K], P[k+2*K]};
    Real Rr[3] = {RX[r], RX[r+Nr], RX[r+2*Nr]};

    // 计算 Rrx
    Real dx = Pk[0] - Rr[0];
    Real dy = Pk[1] - Rr[1];
    Real dz = Pk[2] - Rr[2];
    Real Rrx = sqrtf(dx*dx + dy*dy + dz*dz);

    // 对每个TX计算
    for (int t = 0; t < Nt; t++) {
        Real TXt[3] = {TX[t], TX[t+Nt], TX[t+2*Nt]};
        dx = TXt[0] - Pk[0];
        dy = TXt[1] - Pk[1];
        dz = TXt[2] - Pk[2];
        Real Rtx = sqrtf(dx*dx + dy*dy + dz*dz);

        Real tau = (Rtx + Rrx) / c + delayK[k];
        Real tau_rel = tau - t0_ref;
        int n = round_idx<Real>(tau_rel, fs, round_mode);
        if (n < 0) n = 0;
        if (n >= Lir) continue;

        // 计算衰减权重
        Real w;
        switch (atten_mode) {
            case 0:  // none
                w = A[k];
                break;
            case 1:  // twoway_R
                w = A[k] / (Rtx * Rrx);
                break;
            case 2:  // sqrt_twoway_R
                w = A[k] / (Real)sqrtf((float)(Rtx * Rrx));
                break;
            default:
                w = A[k];
        }

        // 使用原子操作累加到稀疏IR（需要先合并相同索引的元素）
        // 简化：直接使用全局IR，后续在host端转换为稀疏格式
        // 这里保持原实现，在host端处理稀疏化
    }
}

// Kernel 2: 构建IR（每个接收通道）- 保持原实现用于兼容
template<typename Real>
__global__ void build_ir_per_rx(
    const Real* __restrict__ P,      // [K x 3]
    const Real* __restrict__ A,      // [K]
    const Real* __restrict__ TX,     // [Nt x 3]
    const Real* __restrict__ RX,     // [Nr x 3]
    const Real* __restrict__ delayK, // [K]
    Real c,
    Real fs,
    Real t0_ref,
    int K, int Nt, int Nr, int r,
    int round_mode,
    int atten_mode,
    int Lir,
    Real* ir_tx)       // [Lir x Nt]
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= K) return;

    int k = idx;
    Real Pk[3] = {P[k], P[k+K], P[k+2*K]};
    Real Rr[3] = {RX[r], RX[r+Nr], RX[r+2*Nr]};

    // 计算 Rrx
    Real dx = Pk[0] - Rr[0];
    Real dy = Pk[1] - Rr[1];
    Real dz = Pk[2] - Rr[2];
    Real Rrx = sqrtf(dx*dx + dy*dy + dz*dz);

    // 对每个TX计算
    for (int t = 0; t < Nt; t++) {
        Real TXt[3] = {TX[t], TX[t+Nt], TX[t+2*Nt]};
        dx = TXt[0] - Pk[0];
        dy = TXt[1] - Pk[1];
        dz = TXt[2] - Pk[2];
        Real Rtx = sqrtf(dx*dx + dy*dy + dz*dz);

        Real tau = (Rtx + Rrx) / c + delayK[k];
        Real tau_rel = tau - t0_ref;
        int n = round_idx<Real>(tau_rel, fs, round_mode);
        if (n < 0) n = 0;
        if (n >= Lir) continue;

        // 计算衰减权重
        Real w;
        switch (atten_mode) {
            case 0:  // none
                w = A[k];
                break;
            case 1:  // twoway_R
                w = A[k] / (Rtx * Rrx);
                break;
            case 2:  // sqrt_twoway_R
                w = A[k] / (Real)sqrtf((float)(Rtx * Rrx));
                break;
            default:
                w = A[k];
        }

        // 原子累加
        atomicAdd(&ir_tx[n + t*Lir], w);
    }
}

// Kernel 3: 批量卷积（每个接收通道，每个TX）- full卷积
// MATLAB conv(x, h, 'full'): y[n] = sum_k x[k] * h[n-k]
// 其中 n 从 0 到 Mx+Lir-2
template<typename Real>
__global__ void conv_kernel(
    const Real* __restrict__ x,      // [Mx] 激励信号
    const Real* __restrict__ h,      // [Lir] 冲激响应
    Real* __restrict__ y,            // [Mout] 输出
    int Mx, int Lir, int Mout)
{
    int n = blockIdx.x * blockDim.x + threadIdx.x;
    if (n >= Mout) return;

    Real sum = (Real)0.0;
    // 对于full卷积 y[n] = sum_k x[k] * h[n-k]
    // k的范围：max(0, n-Lir+1) <= k <= min(n, Mx-1)
    int k_min = n - (Lir - 1);
    if (k_min < 0) k_min = 0;
    int k_max = (n < Mx - 1) ? n : (Mx - 1);
    
    // 优化：对于大卷积，使用更高效的循环
    // 如果Lir很大，反向遍历h可能更高效（更好的缓存局部性）
    if (Lir > 1024) {
        // 反向遍历h，利用缓存局部性
        for (int h_idx = Lir - 1; h_idx >= 0; h_idx--) {
            int k = n - h_idx;
            if (k >= k_min && k <= k_max && k >= 0 && k < Mx) {
                sum += x[k] * h[h_idx];
            }
        }
    } else {
        // 小卷积：正向遍历k
        for (int k = k_min; k <= k_max; k++) {
            int h_idx = n - k;
            if (h_idx >= 0 && h_idx < Lir) {
                sum += x[k] * h[h_idx];
            }
        }
    }

    y[n] = sum;
}

// Kernel 3b: 批量卷积并累加（优化版本 - 同时处理所有TX并累加）
// 这个kernel同时处理所有TX的卷积并直接累加到输出
// 注意：h_all是列主序存储（MATLAB格式），第t列从 h_all[t*Lir] 开始
// 优化：跳过零值IR元素，利用IR的稀疏性
template<typename Real>
__global__ void conv_and_accumulate_kernel(
    const Real* __restrict__ x_all,  // [Nt x Mx] 所有TX的激励信号（行主序）
    const Real* __restrict__ h_all,   // [Nt x Lir] 所有TX的IR（列主序，MATLAB格式）
    Real* __restrict__ y,             // [Mout] 累加输出
    int Mx, int Lir, int Mout, int Nt)
{
    int n = blockIdx.x * blockDim.x + threadIdx.x;
    if (n >= Mout) return;

    Real sum_total = (Real)0.0;
    
    // 对每个TX计算卷积并累加
    // 注意：IR是列主序，所以第t列从 h_all[t*Lir] 开始（已经是正确的）
    for (int t = 0; t < Nt; t++) {
        const Real* x = x_all + t * Mx;  // 第t行（行主序）
        const Real* h = h_all + t * Lir; // 第t列（列主序）
        
        Real sum = (Real)0.0;
        // 优化：直接遍历k范围，避免反向映射
        // 对于full卷积：y[n] = sum_k x[k] * h[n-k]
        // k的有效范围：max(0, n-Lir+1) <= k <= min(n, Mx-1)
        int k_min = n - (Lir - 1);
        if (k_min < 0) k_min = 0;
        int k_max = (n < Mx - 1) ? n : (Mx - 1);
        
        // 直接遍历k，计算h[n-k]，跳过零值h
        // 这样访问x是顺序的，可能更高效
        for (int k = k_min; k <= k_max; k++) {
            int h_idx = n - k;
            if (h_idx >= 0 && h_idx < Lir) {
                Real h_val = h[h_idx];
                // 跳过零值（利用IR的稀疏性）- 关键优化！
                if (h_val != (Real)0.0) {
                    sum += x[k] * h_val;
                }
            }
        }
        
        sum_total += sum;
    }
    
    y[n] = sum_total;
}

// Kernel 4: 向量累加
__global__ void add_vectors(
    float* __restrict__ y,           // 累加目标
    const float* __restrict__ x,      // 要累加的向量
    int n)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        y[idx] += x[idx];
    }
}

// Kernel: 复数乘法并累加（用于FFT卷积）
__global__ void complex_multiply_accumulate_kernel(
    const float2* __restrict__ x_fft,  // excitation的FFT结果
    const float2* __restrict__ h_fft,   // IR的FFT结果
    float2* __restrict__ y_fft,        // 输出：x_fft * h_fft（累加）
    int N)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;
    
    float2 x_val = x_fft[idx];
    float2 h_val = h_fft[idx];
    
    // 复数乘法：(x.re + i*x.im) * (h.re + i*h.im) = (x.re*h.re - x.im*h.im) + i*(x.re*h.im + x.im*h.re)
    float2 result;
    result.x = x_val.x * h_val.x - x_val.y * h_val.y;
    result.y = x_val.x * h_val.y + x_val.y * h_val.x;
    
    // 累加到输出
    y_fft[idx].x += result.x;
    y_fft[idx].y += result.y;
}

// Kernel: 从复数提取实部并归一化（用于IFFT后的结果）
__global__ void extract_real_part_kernel(
    const float* __restrict__ complex_in,  // IFFT后的实数数组（已经是实部）
    float* __restrict__ real_out,          // 实数输出
    float scale,                            // 归一化因子（1/fft_size）
    int N)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;
    
    // IFFT的结果已经是实数，只需要归一化
    real_out[idx] = complex_in[idx] * scale;
}

// ==================== MEX入口函数 ====================

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    // 总时间测量
    auto t_total_start = std::chrono::high_resolution_clock::now();
    
    // 检查输入参数数量
    if (nrhs != 11) {
        mexErrMsgIdAndTxt("simrx:input", "Need 11 input arguments");
    }
    
    mexPrintf("[PROFILE] ===== CUDA Performance Analysis =====\n");

    // 读取输入
    const mxArray* mxP = prhs[0];
    const mxArray* mxA = prhs[1];
    const mxArray* mxTX = prhs[2];
    const mxArray* mxRX = prhs[3];
    double c = mxGetScalar(prhs[4]);
    double fs = mxGetScalar(prhs[5]);
    const mxArray* mxExcitation = prhs[6];
    int round_mode = (int)mxGetScalar(prhs[7]);
    int atten_mode = (int)mxGetScalar(prhs[8]);
    const mxArray* mxDelayK = prhs[9];
    bool is_single = (bool)mxGetScalar(prhs[10]);

    // 获取维度并验证
    mwSize K = mxGetM(mxP);
    mwSize Nt = mxGetM(mxTX);
    mwSize Nr = mxGetM(mxRX);
    mwSize Mx = mxGetN(mxExcitation);
    
    // 验证维度
    if (K == 0 || Nt == 0 || Nr == 0 || Mx == 0) {
        mexErrMsgIdAndTxt("simrx:input", "Input dimensions must be non-zero");
    }
    if (mxGetN(mxP) != 3 || mxGetN(mxTX) != 3 || mxGetN(mxRX) != 3) {
        mexErrMsgIdAndTxt("simrx:input", "P, TX, RX must have 3 columns");
    }
    if (mxGetM(mxExcitation) != Nt) {
        mexErrMsgIdAndTxt("simrx:input", "excitation must have Nt rows");
    }

    // 读取数据
    const double* P_d = mxGetPr(mxP);
    const double* A_d = mxGetPr(mxA);
    const double* TX_d = mxGetPr(mxTX);
    const double* RX_d = mxGetPr(mxRX);
    const double* excitation_d = mxGetPr(mxExcitation);
    const double* delayK_d = mxGetPr(mxDelayK);

    // 分配GPU内存
    size_t P_size = K * 3 * sizeof(float);
    size_t A_size = K * sizeof(float);
    size_t TX_size = Nt * 3 * sizeof(float);
    size_t RX_size = Nr * 3 * sizeof(float);
    size_t excitation_size = Nt * Mx * sizeof(float);
    size_t delayK_size = K * sizeof(float);

    float *d_P, *d_A, *d_TX, *d_RX, *d_excitation, *d_delayK;
    CHECK_CUDA(cudaMalloc(&d_P, P_size));
    CHECK_CUDA(cudaMalloc(&d_A, A_size));
    CHECK_CUDA(cudaMalloc(&d_TX, TX_size));
    CHECK_CUDA(cudaMalloc(&d_RX, RX_size));
    CHECK_CUDA(cudaMalloc(&d_excitation, excitation_size));
    CHECK_CUDA(cudaMalloc(&d_delayK, delayK_size));

    // 转换并复制到GPU
    PROFILE_START(data_convert);
    // 注意：MATLAB数组是列主序，需要转换为行主序以便CUDA使用
    std::vector<float> P_f(K*3), A_f(K), TX_f(Nt*3), RX_f(Nr*3), 
                       excitation_f(Nt*Mx), delayK_f(K);
    for (mwSize i = 0; i < K*3; i++) P_f[i] = (float)P_d[i];
    for (mwSize i = 0; i < K; i++) A_f[i] = (float)A_d[i];
    for (mwSize i = 0; i < Nt*3; i++) TX_f[i] = (float)TX_d[i];
    for (mwSize i = 0; i < Nr*3; i++) RX_f[i] = (float)RX_d[i];
    
    // excitation: MATLAB [Nt x Mx] 列主序 -> CUDA行主序
    // MATLAB: excitation(j,i) 在位置 j + i*Nt
    // CUDA: 我们希望 excitation[t*Mx + m] 是第t行第m列
    for (mwSize t = 0; t < Nt; t++) {
        for (mwSize m = 0; m < Mx; m++) {
            excitation_f[t*Mx + m] = (float)excitation_d[t + m*Nt];  // 列主序转行主序
        }
    }
    
    for (mwSize i = 0; i < K; i++) delayK_f[i] = (float)delayK_d[i];
    PROFILE_STOP(data_convert);
    PROFILE_PRINT(data_convert, "Data conversion (CPU)");

    PROFILE_START(memcpy_h2d);
    CHECK_CUDA(cudaMemcpy(d_P, P_f.data(), P_size, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(d_A, A_f.data(), A_size, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(d_TX, TX_f.data(), TX_size, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(d_RX, RX_f.data(), RX_size, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(d_excitation, excitation_f.data(), excitation_size, cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(d_delayK, delayK_f.data(), delayK_size, cudaMemcpyHostToDevice));
    PROFILE_STOP(memcpy_h2d);
    PROFILE_PRINT(memcpy_h2d, "Memory copy Host->Device");

    float c_f = (float)c;
    float fs_f = (float)fs;

    // Pass 1: 找到最小最大时延
    float *d_t_min, *d_t_max;
    CHECK_CUDA(cudaMalloc(&d_t_min, sizeof(float)));
    CHECK_CUDA(cudaMalloc(&d_t_max, sizeof(float)));
    
    float t_min_init = 1e30f;
    float t_max_init = -1e30f;
    CHECK_CUDA(cudaMemcpy(d_t_min, &t_min_init, sizeof(float), cudaMemcpyHostToDevice));
    CHECK_CUDA(cudaMemcpy(d_t_max, &t_max_init, sizeof(float), cudaMemcpyHostToDevice));

    int threadsPerBlock = 256;
    int totalBlocks = (K * Nr + threadsPerBlock - 1) / threadsPerBlock;
    if (totalBlocks == 0) totalBlocks = 1;  // 确保至少有一个block
    size_t sharedMemSize = 2 * threadsPerBlock * sizeof(float);
    
    // 检查共享内存大小是否超过限制（通常为48KB）
    cudaDeviceProp prop;
    CHECK_CUDA(cudaGetDeviceProperties(&prop, 0));
    if (sharedMemSize > prop.sharedMemPerBlock) {
        mexErrMsgIdAndTxt("simrx:cuda", "Shared memory size exceeds device limit");
    }
    
    PROFILE_START(kernel_find_range);
    compute_delays_find_range<float><<<totalBlocks, threadsPerBlock, sharedMemSize>>>(
        d_P, d_TX, d_RX, d_delayK, c_f, fs_f, K, Nt, Nr, round_mode,
        d_t_min, d_t_max);
    PROFILE_STOP(kernel_find_range);
    PROFILE_PRINT(kernel_find_range, "Kernel: find delay range");

    float t_min, t_max;
    CHECK_CUDA(cudaMemcpy(&t_min, d_t_min, sizeof(float), cudaMemcpyDeviceToHost));
    CHECK_CUDA(cudaMemcpy(&t_max, d_t_max, sizeof(float), cudaMemcpyDeviceToHost));

    float t0_ref = t_min;
    
    // 计算Lir
    float tau_range = t_max - t0_ref;
    int Lir;
    switch (round_mode) {
        case 0: Lir = (int)roundf(tau_range * fs_f) + 1; break;
        case 1: Lir = (int)floorf(tau_range * fs_f) + 1; break;
        case 2: Lir = (int)ceilf(tau_range * fs_f) + 1; break;
        default: Lir = (int)roundf(tau_range * fs_f) + 1;
    }
    if (Lir < 1) Lir = 1;

    int Mout = Lir + Mx - 1;
    if (Mout < 1) Mout = 1;  // 确保输出大小至少为1

    // 分配输出内存
    mxClassID outClass = is_single ? mxSINGLE_CLASS : mxDOUBLE_CLASS;
    plhs[0] = mxCreateNumericMatrix(Mout, Nr, outClass, mxREAL);
    plhs[1] = mxCreateDoubleScalar((double)t0_ref);

    // 始终在GPU上分配内存，最后再复制回MATLAB数组
    float *d_y;
    CHECK_CUDA(cudaMalloc(&d_y, Mout * Nr * sizeof(float)));
    CHECK_CUDA(cudaMemset(d_y, 0, Mout * Nr * sizeof(float)));

    // Pass 2: 对每个接收通道构建IR并卷积
    float *d_ir_tx;
    CHECK_CUDA(cudaMalloc(&d_ir_tx, Lir * Nt * sizeof(float)));

    // 预分配临时缓冲区（避免循环中分配/释放）
    float *d_yr;
    CHECK_CUDA(cudaMalloc(&d_yr, Mout * sizeof(float)));
    // 注意：不再需要d_yr_temp，因为使用批量卷积kernel直接累加

    int threadsK = 256;
    int blocksK = (K + threadsK - 1) / threadsK;
    if (blocksK == 0) blocksK = 1;
    float total_build_ir = 0.0f, total_conv = 0.0f, total_sync = 0.0f, total_memcpy = 0.0f;
    
    // 预分配FFT缓冲区（避免每次RX都重新分配）
    // 计算FFT长度（需要至少Mout，最好是2的幂次）
    int fft_size = 1;
    while (fft_size < Mout) {
        fft_size *= 2;
    }
    
    // 创建FFT计划
    cufftHandle plan_forward, plan_inverse;
    CHECK_CUFFT(cufftPlan1d(&plan_forward, fft_size, CUFFT_R2C, 1));
    CHECK_CUFFT(cufftPlan1d(&plan_inverse, fft_size, CUFFT_C2R, 1));
    
    // 分配FFT缓冲区（在循环外分配，避免重复分配）
    float *d_x_padded, *d_h_padded, *d_y_padded;
    float2 *d_x_fft, *d_h_fft, *d_y_fft;
    CHECK_CUDA(cudaMalloc(&d_x_padded, fft_size * sizeof(float)));
    CHECK_CUDA(cudaMalloc(&d_h_padded, fft_size * sizeof(float)));
    CHECK_CUDA(cudaMalloc(&d_y_padded, fft_size * sizeof(float)));
    CHECK_CUDA(cudaMalloc(&d_x_fft, (fft_size/2+1) * sizeof(float2)));
    CHECK_CUDA(cudaMalloc(&d_h_fft, (fft_size/2+1) * sizeof(float2)));
    CHECK_CUDA(cudaMalloc(&d_y_fft, (fft_size/2+1) * sizeof(float2)));
    
    for (mwSize r = 0; r < Nr; r++) {
        // 清零IR
        CHECK_CUDA(cudaMemset(d_ir_tx, 0, Lir * Nt * sizeof(float)));

        // 构建IR
        PROFILE_START(kernel_build_ir);
        build_ir_per_rx<float><<<blocksK, threadsK>>>(
            d_P, d_A, d_TX, d_RX, d_delayK, c_f, fs_f, t0_ref,
            K, Nt, Nr, r, round_mode, atten_mode, Lir, d_ir_tx);
        PROFILE_STOP(kernel_build_ir);
        total_build_ir += time_kernel_build_ir;

        // 清零输出缓冲区
        CHECK_CUDA(cudaMemset(d_yr, 0, Mout * sizeof(float)));

        // 优化：使用cuFFT进行FFT卷积（对于大卷积更高效）
        // FFT卷积：y = IFFT(FFT(x) * FFT(h))
        PROFILE_START(kernel_conv);
        
        // 清零y_fft（用于累加）
        CHECK_CUDA(cudaMemset(d_y_fft, 0, (fft_size/2+1) * sizeof(float2)));
        
        // 对每个TX进行FFT卷积并累加
        for (int t = 0; t < Nt; t++) {
            // 零填充excitation
            CHECK_CUDA(cudaMemset(d_x_padded, 0, fft_size * sizeof(float)));
            CHECK_CUDA(cudaMemcpy(d_x_padded, &d_excitation[t*Mx], Mx * sizeof(float), cudaMemcpyDeviceToDevice));
            
            // 零填充IR
            CHECK_CUDA(cudaMemset(d_h_padded, 0, fft_size * sizeof(float)));
            CHECK_CUDA(cudaMemcpy(d_h_padded, &d_ir_tx[t*Lir], Lir * sizeof(float), cudaMemcpyDeviceToDevice));
            
            // FFT: x -> X
            CHECK_CUFFT(cufftExecR2C(plan_forward, d_x_padded, d_x_fft));
            
            // FFT: h -> H
            CHECK_CUFFT(cufftExecR2C(plan_forward, d_h_padded, d_h_fft));
            
            // 在频域相乘并累加：Y += X * H
            int threadsFFT = 256;
            int blocksFFT = ((fft_size/2+1) + threadsFFT - 1) / threadsFFT;
            complex_multiply_accumulate_kernel<<<blocksFFT, threadsFFT>>>(
                d_x_fft, d_h_fft, d_y_fft, fft_size/2+1);
        }
        
        // IFFT: Y -> y
        CHECK_CUFFT(cufftExecC2R(plan_inverse, d_y_fft, d_y_padded));
        
        // 提取前Mout个元素并归一化
        float scale = 1.0f / fft_size;
        int threadsExtract = 256;
        int blocksExtract = (Mout + threadsExtract - 1) / threadsExtract;
        extract_real_part_kernel<<<blocksExtract, threadsExtract>>>(
            d_y_padded, d_yr, scale, Mout);
        
        PROFILE_STOP(kernel_conv);
        total_conv += time_kernel_conv;

        // 只在所有TX处理完后同步一次
        PROFILE_START(sync);
        CHECK_CUDA(cudaDeviceSynchronize());
        PROFILE_STOP(sync);
        total_sync += time_sync;

        // 复制到输出
        PROFILE_START(memcpy_d2d);
        CHECK_CUDA(cudaMemcpy(&d_y[r*Mout], d_yr, Mout * sizeof(float), cudaMemcpyDeviceToDevice));
        PROFILE_STOP(memcpy_d2d);
        total_memcpy += time_memcpy_d2d;
    }
    
    mexPrintf("[PROFILE] Per-RX totals (over %d RX):\n", (int)Nr);
    mexPrintf("[PROFILE]   Build IR: %.3f ms (avg: %.3f ms/RX)\n", total_build_ir, total_build_ir/Nr);
    mexPrintf("[PROFILE]   Convolution: %.3f ms (avg: %.3f ms/RX, %.3f ms/TX)\n", 
              total_conv, total_conv/Nr, total_conv/(Nr*Nt));
    mexPrintf("[PROFILE]   Synchronization: %.3f ms (avg: %.3f ms/RX)\n", total_sync, total_sync/Nr);
    mexPrintf("[PROFILE]   Memory copy D2D: %.3f ms (avg: %.3f ms/RX)\n", total_memcpy, total_memcpy/Nr);

    // 释放临时缓冲区
    CHECK_CUDA(cudaFree(d_yr));

    // 将GPU结果复制回MATLAB数组
    PROFILE_START(memcpy_d2h);
    if (is_single) {
        float *y_f = (float*)mxGetData(plhs[0]);
        CHECK_CUDA(cudaMemcpy(y_f, d_y, Mout * Nr * sizeof(float), cudaMemcpyDeviceToHost));
    } else {
        double *y_d = (double*)mxGetData(plhs[0]);
        std::vector<float> y_f(Mout * Nr);
        CHECK_CUDA(cudaMemcpy(y_f.data(), d_y, Mout * Nr * sizeof(float), cudaMemcpyDeviceToHost));
        for (mwSize i = 0; i < Mout * Nr; i++) {
            y_d[i] = (double)y_f[i];
        }
    }
    PROFILE_STOP(memcpy_d2h);
    PROFILE_PRINT(memcpy_d2h, "Memory copy Device->Host");
    CHECK_CUDA(cudaFree(d_y));

    // 清理
    CHECK_CUDA(cudaFree(d_P));
    CHECK_CUDA(cudaFree(d_A));
    CHECK_CUDA(cudaFree(d_TX));
    CHECK_CUDA(cudaFree(d_RX));
    CHECK_CUDA(cudaFree(d_excitation));
    CHECK_CUDA(cudaFree(d_delayK));
    CHECK_CUDA(cudaFree(d_t_min));
    CHECK_CUDA(cudaFree(d_t_max));
    CHECK_CUDA(cudaFree(d_ir_tx));
    
    // 清理FFT缓冲区和计划
    CHECK_CUDA(cudaFree(d_x_padded));
    CHECK_CUDA(cudaFree(d_h_padded));
    CHECK_CUDA(cudaFree(d_y_padded));
    CHECK_CUDA(cudaFree(d_x_fft));
    CHECK_CUDA(cudaFree(d_h_fft));
    CHECK_CUDA(cudaFree(d_y_fft));
    CHECK_CUFFT(cufftDestroy(plan_forward));
    CHECK_CUFFT(cufftDestroy(plan_inverse));
    
    // 总时间
    auto t_total_end = std::chrono::high_resolution_clock::now();
    auto t_total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        t_total_end - t_total_start).count();
    
    mexPrintf("[PROFILE] ===== Summary =====\n");
    mexPrintf("[PROFILE] Total time: %.3f ms\n", (float)t_total_ms);
    mexPrintf("[PROFILE] Problem size: K=%zu, Nt=%zu, Nr=%zu, Mx=%zu, Lir=%d\n", 
              (size_t)K, (size_t)Nt, (size_t)Nr, (size_t)Mx, Lir);
    mexPrintf("[PROFILE] ====================\n");
}

