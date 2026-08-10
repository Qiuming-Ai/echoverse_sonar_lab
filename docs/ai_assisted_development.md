# AI-Assisted Development Disclosure

## 1. Confirmed Use

Generative-AI assistance has been used in the development and documentation of
EchoVerse Sonar Lab.

During the early development stage, DeepSeek V4 Pro Token together with VS Code
Copilot was used to assist with drafting GUI and UX code. The product label recorded
by the author is disclosed as provided; a more detailed session-by-session model and
prompt log was not retained.

For the 2026-08-10 revision work, OpenAI Codex was used to:

- compare and synchronize an author-verified Linux implementation with the main tree;
- help identify dependency and cross-platform documentation gaps;
- draft performance-instrumentation code and experiment documentation;
- reorganize and clarify C++/MATLAB responsibility boundaries;
- draft README, architecture, build, scalability, and reviewer-tracking documentation.

This document does not infer or invent tool use that the maintainers have not
confirmed. Maintainers should append any additional tools, model versions, dates, and
tasks to the log below when known.

## 2. Human Oversight and Verification

AI suggestions are not accepted as evidence of correctness. Human maintainers remain
responsible for:

- architecture and scientific-model choices;
- reviewing source changes and third-party licenses;
- compiling on the declared operating systems;
- running GUI, file-format, MATLAB, CUDA, and numerical checks;
- interpreting performance results and deciding what is reported in a manuscript;
- approving releases and publication text.

The Linux main-camera changes synchronized during this revision were based on files
that an author had already compiled and run on Ubuntu 24.04. The merged repository
subsequently passed a Windows/vcpkg Release build and a two-frame runtime smoke test
that produced the expected performance CSV. A Linux clean-clone build and longer
cross-platform regression run are still required before release.

## 3. Tasks AI Did Not Replace

AI was not treated as an author, reviewer, or autonomous scientific validator. AI
output must not be used to fabricate benchmarks, references, experimental results, or
platform-support claims. The core C++ sonar-image generation functionality and the
MATLAB waveform-generation functionality were designed and implemented by the author,
not delegated to the AI tools listed above. Performance data must come from the
opt-in profilers and executed runs.

## 4. Verification Procedure for AI-Assisted Code

For each AI-assisted code change, maintainers should retain evidence of:

1. source review or diff review;
2. a clean configure and build;
3. a runtime smoke test on each claimed platform;
4. targeted functional or numerical checks;
5. confirmation that generated files and dependencies have compatible licenses;
6. a final human decision to accept, revise, or reject the suggestion.

For CUDA work, verification must also confirm that
`sim_rx_from_scatterers_perTX_cuda_mex.cu` was compiled by MATLAB, that the MEX binary
is on the MATLAB path, and that `EchoInit` reports the `cuda_mex` backend.

## 5. Development-Use Log

| Date | Tool | Scope | Human verification status |
|---|---|---|---|
| Early development, exact dates not retained | DeepSeek V4 Pro Token with VS Code Copilot | Assisted drafting of GUI and UX code | Author reviewed and integrated the code; core sonar-image and MATLAB waveform-generation functions remained author-implemented |
| 2026-08-10 | OpenAI Codex | Linux-change integration, Eigen submodule setup, performance logging, repository and manuscript documentation, reviewer TODO | Windows/vcpkg Release build passed; two prepared scenes completed three 100-frame C++ runs each; author had previously run the supplied Ubuntu variant; Linux clean-clone regression remains pending |

## 6. Suggested Manuscript Disclosure

If AI-assisted code development is material to the research software, describe it in
the manuscript's software-development or methods section, including the tool, purpose,
and human verification. If generative AI was also used for substantive manuscript
preparation, add the journal-required declaration immediately before the references.

The manuscript statement must be reviewed by all authors and must reflect actual use;
this repository document is not a substitute for the publisher's declaration.

Suggested wording:

> During software development, the authors used DeepSeek V4 Pro Token together with
> VS Code Copilot to assist in drafting GUI and UX code. The core C++ sonar-image
> generation functionality and MATLAB waveform-generation functionality were designed
> and implemented by the authors. OpenAI Codex was subsequently used to assist with
> cross-platform integration, performance instrumentation, documentation, and revision
> organization. All AI-assisted outputs were reviewed, edited, built, and tested by
> the authors, who take full responsibility for the software and manuscript.
