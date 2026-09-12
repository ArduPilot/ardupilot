# MCUBoot A/B on Zephyr, and the state of bootloader security

MCUBoot is the Zephyr project's own bootloader, and its image format is what
the whole Zephyr update ecosystem speaks. AP_Bootloader on Zephyr implements
MCUBoot's A/B update path against that format exactly: what `imgtool` signs is
what this bootloader parses, and mcumgr over the SMP server delivers it. There
is no private variant and no ArduPilot-only step in that chain.

The A/B path checks integrity but does not yet check authenticity, as doing
so requires a cypher we dont yet have.

## What the bootloader does check

`Tools/AP_Bootloader/mcuboot_ab.cpp` implements overwrite-only A/B. An image
staged in slot 1 is walked as a real MCUBoot image, so the magic, the hashed
span and the TLV trailer all have to be right, and its SHA256 TLV has to match
what the bootloader computes before the image is copied over slot 0.

That is an integrity check. It catches a truncated or corrupted upload. It
does not establish who produced the image, so anything that can write slot 1
can get its image adopted.

## What is missing

The ED25519 TLV that `imgtool sign` writes is not verified. Doing so needs an
RFC 8032 Ed25519 verifier, and the monocypher in this tree is the Blake2b
flavour that ArduPilot secure boot uses, which cannot check an imgtool
signature.

Two Ed25519 flavours are involved and they are not interchangeable:

| | ArduPilot secure boot | MCUBoot / imgtool |
| --- | --- | --- |
| Flavour | Ed25519 with Blake2b | RFC 8032 Ed25519, SHA-512 |
| Signature lives in | `.apj`, APSE block | image TLV trailer |
| Key file format | `PRIVATE_KEYV1:` and base64 | PKCS#8 PEM |

The same 32-byte seed therefore yields two different public keys, one per
flavour, which is the trap to remember whenever this work restarts.

## Until then

Treat A/B as an integrity-checked update path, not a secure-boot one, and do
not describe these boards as having verified boot. Adding the verifier, the
key provisioning and the trust store belongs in its own change where it can
be reviewed as security work rather than as part of a HAL port.
