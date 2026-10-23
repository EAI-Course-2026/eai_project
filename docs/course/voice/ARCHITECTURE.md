# Voice interaction architecture

This document separates the provider-neutral voice workflow from the current
course implementation. It is a design boundary, not a claim that provider
selection or a typed event protocol has already been implemented.

## Goal

A user should be able to clone this repository, choose a speech recognition
provider that fits their device and preferences, and connect it to the same
safe command and robot execution path. The application repository must not
require a particular model, cloud vendor, local checkout, or audio stack.

The speech provider converts audio to recognition events. It does not choose
robot actions. Only the repository's command policy can turn an accepted,
final transcript into a command from the fixed allowlist.

## Components and ownership

```text
Microphone / audio source
        |
        v
Speech provider adapter (local model, hosted API, or another device)
        |
        v
Provider-neutral recognition events
        |
        v
Voice input transport (stdin today; other transports can be added)
        |
        v
Allowlist intent policy -> pose plan -> explicit execution gate
        |
        v
Robot controller -> device-specific hardware configuration
```

| Component | Owns | Must not own |
|---|---|---|
| Speech provider | Audio capture/transport, recognition, provider credentials and provider errors | Robot command policy or motor control |
| Input adapter | Converts a provider's output into the voice application's input contract | Provider-specific model logic or robot credentials |
| Voice command layer | Allowlist, rejection rules, planning, stop preemption and event records | Audio capture, model loading or provider selection |
| Robot execution | Serial connection, calibration, poses, motion enable and hardware checks | Speech provider selection |

The robot process should remain close to the robot and own the hardware safety
checks. Recognition may run locally or remotely. A remote provider needs a
client/adapter on the user's device; the robot command layer should not need to
load a vendor SDK just to receive text.

Keeping providers external also keeps optional audio libraries, large model
dependencies, service credentials and provider-specific privacy choices out of
the shared robot control environment. This is an architectural reason to keep
the boundary; it is not a requirement to use local inference.

## Current implementation

- `eai-course voice` accepts one text with `--text`, or reads UTF-8 text from
  standard input, one line per completed transcript, with `--stdin`.
- The standard-input boundary is provider-neutral. A local recognizer, a
  hosted-service client, or a bridge on another device can write to it.
- `edge_inference_lab` / `mic_asr.py` is one integration example used for the
  2026-10-12 software dry run. It is not installed or imported by this repo.
- The current line protocol has no provider identity, confidence, partial/final
  marker, timestamp, or structured error event. Each non-empty line is treated
  as a final transcript. Do not infer provider portability validation from the
  one tested example.
- Hardware options remain machine-specific: serial port, calibration and pose
  files describe the connected robot, not the speech service. They are
  intentionally configured separately.

## Provider-neutral event contract (evolution target)

When the plain-text stream becomes insufficient, add an adapter boundary in
`applications/teleoperation/` rather than adding provider SDK calls to
`voice.py` or `intent.py`. The normalized event should carry only the data
needed by the voice workflow:

| Field | Meaning |
|---|---|
| `type` | `partial`, `final`, `error`, or `end` |
| `text` | Transcript text for `partial` or `final`; absent otherwise |
| `provider` | Optional provider identifier for diagnostics |
| `confidence` | Optional provider score; its scale and meaning remain provider-specific |
| `timestamp` | Optional event time for latency measurement and audit |

Only `final` events may enter the intent parser. Partial transcripts, provider
errors and stream termination must never become commands. Preserve a
high-priority stop path so a recognized stop can preempt queued ordinary
commands. Provider confidence scores are not assumed comparable; any threshold
or confirmation policy must be explicit and fail closed when required data is
missing.

The first implementation can continue to use standard input as a transport.
A versioned JSON-lines event format or an in-process adapter can be added when
there is a concrete need for metadata or lifecycle handling. Keep the current
one-line text mode compatible during that transition.

## Configuration and portability

Provider selection belongs to the user's runtime configuration, not shared
defaults tied to one workstation. A future example configuration may describe
provider name, model, endpoint and audio options; secrets must come from
environment variables or an operating-system credential store. Never commit
API keys, personal paths, audio, or model weights.

Keep speech settings separate from `hardware.local.toml` and robot calibration:

- the speech provider is selected per user/device and may change independently;
- robot port, calibration and pose files are selected for the physical robot;
- a clone can use the command layer in dry-run mode without either a speech
  provider or a serial device.

Future adapters that call a network/provider service must stop accepting new
commands after a provider failure. They must not replay old transcripts or
resume a prior motion plan. Voice `stop` is a software preemption command; it
is not a substitute for a physical emergency stop.

## Implementation sequence

1. Keep the generic stdin contract and document provider choice as external.
2. Add a small input-provider interface and typed recognition events when
   metadata/lifecycle support is needed; preserve stdin text compatibility.
3. Add user-local provider configuration and one adapter at a time, with mock
   events for offline development.
4. Validate each provider's latency, failure behavior, language/noise coverage
   and stop responsiveness separately from dry-run and real-robot acceptance.

## Agent input integration (2026-10-19)

`eai-course agent --stdin --provider-config FILE` now accepts the same final-text
line transport for LLM tool selection, with an independent local stop path and
dry-run tools only. `eai-course voice` keeps the deterministic command policy.
The chat provider config does not select an ASR model, capture audio, or replace
robot hardware settings. Structured recognition events and local speech-provider
configuration above remain future work. See the [Agent runtime](../../agent_runtime.md).

## Desktop cloud speech (2026-10-24)

The existing arm desktop now embeds an Agent chat view with explicit microphone
capture, cloud ASR, the existing text/tool harness and cloud TTS playback.
Chat, ASR and TTS have independent browser-session API settings held in server RAM;
file profiles remain optional operator/CLI compatibility. ASR/TTS use the official DashScope SDK;
they do not depend on a local inference project or replace stdin providers.
Only final transcripts enter the harness. This first version uploads a completed
recording; it is not continuous listening or Omni. Tools remain dry-run.
See the [startup and configuration guide](../../speech_agent.md) and
[real API evidence](../../reports/2026-10-24-cloud-speech.md).
