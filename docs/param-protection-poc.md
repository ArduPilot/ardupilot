# ArduPlane Parameter Protection PoC

This branch is a proof of concept for protecting proprietary ArduPlane tuning
parameters from unsigned MAVLink extraction and from raw SD-card log inspection.
All runtime behavior is gated behind compile-time flags:

- `AP_PARAM_PROTECTION_ENABLED`
- `AP_LOGGER_ENCRYPTION_ENABLED`

The branch intentionally does not touch RDP, option bytes, secure boot, signed
bootloaders, or board flash protection.

## PoC Decisions

The following values are generated/demo decisions for SITL validation only.
They are not production provisioning decisions.

- Protected parameters:
  - Exact names: `LOG_BITMASK`, `TSV_SECRET_GAIN`
  - Prefix: `TSV_TUNE_`
- MAVLink authorization model: MAVLink2 signing is the trust signal.
- Unsigned protected writes: blocked.
- Log handling: `ENCRYPT-AND-KEEP`; protected parameters remain in logs, but the
  SD-card bytes are encrypted.
- Log key provisioning: development public X25519 key compiled into SITL/dev
  builds via `AP_LOGGER_ENCRYPTION_PUBLIC_KEY`.
- Private key custody: offline host only. The private key is never compiled into
  firmware and is needed only by `Tools/scripts/ap_logger_decrypt.py`.

Production builds should replace the development public key at build time and
choose whether it is provisioned per fleet or per device.

## Behavior

When `AP_PARAM_PROTECTION_ENABLED=1`, unsigned MAVLink parameter clients cannot
list, read, or set protected parameters. A signed MAVLink2 client with the
configured signing key can still list/read protected parameters.

When `AP_LOGGER_ENCRYPTION_ENABLED=1`, the file and block logger write streams
are envelope-encrypted:

- A random per-log data key is generated on the aircraft.
- Each log frame is encrypted with XChaCha20-Poly1305 and a unique frame nonce.
- The data key is wrapped with a build-time public X25519 key and stored in the
  encrypted log header.
- The matching private key is required offline to decrypt the log into a
  standard ArduPilot `.BIN`.

This follows the envelope approach used by PX4 encrypted logging at the scheme
level, without copying PX4 code.

## Verification Status

Goal status for this PoC branch:

- G0 Baseline: PASS
- G1 Protected-parameter helper: PASS
- G2 Unsigned dump hides protected params: PASS
- G3 Signed dump sees protected params: PASS
- G4 Unsigned protected read/set blocked: PASS
- G5 Logs not readable without private key: PASS via encrypted-in-log strategy
- G6 Flag-off regression: PASS
- G7 Envelope log encryption in SITL: PASS
- G8 Offline decrypt tool: PASS

Verification performed on branch `param-protection`:

```sh
PATH=.venv/bin:$PATH ./waf configure --board sitl \
  --define=AP_PARAM_PROTECTION_ENABLED=1 \
  --define=AP_LOGGER_ENCRYPTION_ENABLED=1
PATH=.venv/bin:$PATH ./waf plane
```

SITL produced an encrypted log whose raw bytes started with `APLOGE1` and did
not contain plaintext `LOG_BITMASK`, `TSV_SECRET_GAIN`, or `TSV_TUNE_`.

The offline decrypt tool reproduced a standard ArduPilot `.BIN`:

```sh
.venv/bin/python Tools/scripts/ap_logger_decrypt.py \
  /path/to/encrypted.BIN \
  /path/to/decrypted.BIN \
  --private-key-hex 77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a

.venv/bin/python Tools/scripts/mavlogdump.py /path/to/decrypted.BIN \
  --types FMT --no-timestamps
```

`mavlogdump.py` parsed the decrypted output successfully. The decrypted output
contained `LOG_BITMASK`, confirming the PoC keeps protected parameters in the
log but makes the raw on-card file unreadable without the private key.

Flag-off regression was also verified:

```sh
PATH=.venv/bin:$PATH ./waf configure --board sitl
PATH=.venv/bin:$PATH ./waf plane
.venv/bin/python Tools/autotest/param_protection_check.py \
  --endpoint tcp:127.0.0.1:5760 \
  --protected LOG_BITMASK \
  unsigned-visible-list
```

The raw flag-off log contained plaintext `LOG_BITMASK`, matching stock behavior.

## Production Handoff

Before production use, replace the PoC decisions with deployment-specific
choices:

- Final protected parameter names and prefixes.
- Signing-key model: shared key, per-device key, or secure element.
- Log public-key provisioning: per fleet or per device.
- Offline private-key custody and recovery policy.
- Hardware validation over real USB with Mission Planner/QGC.
- Secure boot and RDP/option-byte flow, handled outside this branch.
