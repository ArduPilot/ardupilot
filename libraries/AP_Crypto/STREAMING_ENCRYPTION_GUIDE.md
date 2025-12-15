# Streaming Encryption for Large Files

## Overview

The simple XOR encryption supports **streaming** for large files like log files. This allows:
- ✅ Encryption/decryption of files larger than available RAM
- ✅ Processing data in chunks (no full-file buffering)
- ✅ Support for multi-GB log files
- ✅ USB streaming log downloads

## Architecture

### For Log Files (SD Card)

**File Format**: Just ciphertext (no header, no MAC)
```
[Ciphertext: variable length]
```

**Streaming Process**:
1. Initialize encryption context with key
2. For each chunk of log data:
   - Generate keystream block (SHA256(key + counter))
   - XOR chunk with keystream
   - Write encrypted chunk to file
   - Increment counter
3. Continue until all data written

### For USB Log Downloads

**Streaming Process**:
1. Initialize decryption context with key
2. Read encrypted log file in chunks
3. For each chunk:
   - Generate keystream block (SHA256(key + counter))
   - XOR chunk with keystream
   - Send decrypted chunk over USB/MAVLink
   - Increment counter
4. Continue until file fully downloaded

## Implementation

### C++ Streaming API

**Header**: `AP_Crypto_Simple_Streaming.h`

**Encryption**:
```cpp
SimpleXORStreamEncrypt ctx;
AP_Crypto_Simple_Streaming::streaming_encrypt_init(&ctx, key);

// For each chunk:
ssize_t encrypted = AP_Crypto_Simple_Streaming::streaming_encrypt_write(
    &ctx, plaintext_chunk, chunk_size, ciphertext_out, max_size);

// When done:
AP_Crypto_Simple_Streaming::streaming_encrypt_cleanup(&ctx);
```

**Decryption**:
```cpp
SimpleXORStreamDecrypt ctx;
AP_Crypto_Simple_Streaming::streaming_decrypt_init(&ctx, key);

// For each chunk:
ssize_t decrypted = AP_Crypto_Simple_Streaming::streaming_decrypt_read(
    &ctx, ciphertext_chunk, chunk_size, plaintext_out, max_size);

// When done:
AP_Crypto_Simple_Streaming::streaming_decrypt_cleanup(&ctx);
```

## Integration Points

### 1. Log File Writing (`AP_Logger_File.cpp`)

**Current**: Uses `AP_Crypto::streaming_encrypt_*` (ChaCha20-Poly1305)

**New Option**: Use `AP_Crypto_Simple_Streaming::streaming_encrypt_*` (Simple XOR)

**Location**: `libraries/AP_Logger/lua_encrypted_log_writer.cpp`

**Modification**: Add support for simple XOR format alongside existing format

### 2. USB Log Downloads (`AP_Logger_MAVLinkLogTransfer.cpp`)

**Current**: Uses `AP_Crypto::streaming_decrypt_*` (ChaCha20-Poly1305)

**New Option**: Use `AP_Crypto_Simple_Streaming::streaming_decrypt_*` (Simple XOR)

**Location**: `libraries/AP_Logger/AP_Logger_MAVLinkLogTransfer.cpp`

**Modification**: Detect format and use appropriate decryption

## File Format Detection

**How to detect simple XOR format**:
- No magic header (unlike "ELOG" or "ELUA")
- No base64url encoding
- Just raw ciphertext
- File extension: `.BIN` (same as regular logs)

**Detection logic**:
```cpp
// Check if file is simple XOR format:
// 1. File doesn't start with known magic headers
// 2. File size > 0
// 3. LEIGH_CRYPT_LEVEL indicates encryption enabled
// 4. Use simple XOR decryption
```

## Python Decryption (Large Files)

**Streaming decryption script**:
```python
def decrypt_file_streaming(input_file, output_file, leigh_key_value):
    """Decrypt large file in chunks."""
    key = derive_key_from_leigh_key(leigh_key_value)
    
    counter = 0
    chunk_size = 64 * 1024  # 64KB chunks
    
    with open(input_file, 'rb') as fin, open(output_file, 'wb') as fout:
        while True:
            ciphertext_chunk = fin.read(chunk_size)
            if not ciphertext_chunk:
                break
            
            # Decrypt chunk
            plaintext_chunk = decrypt_chunk(ciphertext_chunk, key, counter)
            fout.write(plaintext_chunk)
            
            # Update counter based on full blocks
            counter += (len(ciphertext_chunk) + 31) // 32
```

## Memory Usage

**Simple XOR Streaming**:
- **Context**: ~50 bytes (key + counter + state)
- **Chunk buffer**: Configurable (typically 4KB-64KB)
- **Total**: < 100KB regardless of file size

**Comparison**:
- **Buffered**: Requires full file in memory (GBs for large logs)
- **Streaming**: Constant memory usage (< 100KB)

## Performance

**Simple XOR**:
- **Keystream generation**: SHA256 per 32-byte block (~1-5μs per block)
- **XOR operation**: Very fast (~1 cycle per byte)
- **Throughput**: ~10-50 MB/s (depends on SHA256 speed)

**For 1GB log file**:
- Blocks: 1GB / 32 = 33,554,432 blocks
- SHA256 calls: 33,554,432
- Estimated time: 30-60 seconds (depends on CPU)

## Integration Checklist

- [ ] Add `AP_Crypto_Simple_Streaming` to build system
- [ ] Modify `log_encrypted_writer_init()` to support simple XOR
- [ ] Modify `log_encrypted_writer_write()` to use simple XOR streaming
- [ ] Modify USB log download to detect and decrypt simple XOR format
- [ ] Add format detection logic
- [ ] Test with small files (< 1MB)
- [ ] Test with medium files (10-100MB)
- [ ] Test with large files (> 1GB)
- [ ] Test USB streaming downloads
- [ ] Verify memory usage stays constant

## Notes

### Counter Continuity

**Important**: The counter must be continuous across chunks:
- Chunk 1: counter = 0, 1, 2, ... (for each 32-byte block)
- Chunk 2: counter = N, N+1, N+2, ... (where N = blocks in chunk 1)
- This ensures keystream is unique for each byte position

### No MAC

**Security Note**: Simple XOR format has no MAC (authentication). This means:
- ✅ Encryption works (confidentiality)
- ❌ No integrity verification (can't detect tampering)
- ⚠️ Suitable for basic obfuscation, not high-security

### SHA-256 Implementation

**Current**: Uses BLAKE2b (from Monocypher) in C++
**Python**: Uses SHA-256 (from hashlib)

**For compatibility**: Implement SHA-256 in C++ to match Python exactly.

---

**Status**: ✅ Streaming API created, needs integration into log file writer and USB download


