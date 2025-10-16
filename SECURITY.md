# Security Policy

## Supported Versions

ArduPilot maintains security updates for the following versions:

| Version | Supported          |
| ------- | ------------------ |
| Master  | :white_check_mark: |
| Latest Stable | :white_check_mark: |
| Previous Stable | :white_check_mark: |
| Older versions | :x: |

## Reporting a Vulnerability

**Please do not report security vulnerabilities through public GitHub issues.**

### Reporting Process

If you discover a security vulnerability in ArduPilot, please report it by emailing:

**security@ardupilot.org**

Please include the following information in your report:

- Type of vulnerability (e.g., buffer overflow, SQL injection, cross-site scripting, etc.)
- Full paths of source file(s) related to the manifestation of the vulnerability
- The location of the affected source code (tag/branch/commit or direct URL)
- Any special configuration required to reproduce the issue
- Step-by-step instructions to reproduce the issue
- Proof-of-concept or exploit code (if possible)
- Impact of the issue, including how an attacker might exploit it

### What to Expect

- **Acknowledgment**: We will acknowledge receipt of your vulnerability report within 48 hours
- **Assessment**: We will assess the vulnerability and determine its severity within 7 days
- **Updates**: We will provide regular updates on our progress (at least every 14 days)
- **Resolution**: We aim to resolve critical vulnerabilities within 90 days
- **Disclosure**: We will coordinate public disclosure with you after a fix is available

### Coordinated Disclosure

We follow a coordinated disclosure process:

1. **Private disclosure** to the ArduPilot development team
2. **Development** of a fix in a private repository
3. **Testing** and validation of the fix
4. **Release** of the patched version
5. **Public disclosure** with credit to the reporter (if desired)

## Security Best Practices

### For Users

- Always run the latest stable version
- Keep firmware updated regularly
- Use secure communication channels (encrypted telemetry)
- Implement proper authentication mechanisms
- Follow the security guidelines in the documentation
- Monitor security advisories regularly

### For Developers

- Review the [Secure Coding Guidelines](https://ardupilot.org/dev/docs/secure-coding.html)
- Use static analysis tools before submitting PRs
- Validate all inputs and parameters
- Implement proper error handling
- Use memory-safe practices
- Review security implications of changes
- Sign your commits with GPG keys

## Security Features

ArduPilot implements several security features:

- **MAVLink authentication**: Optional message authentication
- **Signed firmware**: Cryptographic verification of firmware
- **Parameter protection**: Read-only and restricted parameters
- **Geo-fencing**: Enforced flight boundaries
- **Failsafe mechanisms**: Automatic safety responses
- **Input validation**: Comprehensive input sanitization

## Security Advisories

Security advisories are published at:
- GitHub Security Advisories: https://github.com/ArduPilot/ardupilot/security/advisories
- ArduPilot Website: https://ardupilot.org/security

Subscribe to security notifications to stay informed.

## Security Scanning

Our CI/CD pipeline includes:

- **SAST**: Static Application Security Testing with CodeQL and Semgrep
- **Dependency Scanning**: Automated vulnerability detection in dependencies
- **Container Scanning**: Security analysis of Docker images
- **Secret Detection**: Prevention of credential leaks

## Bug Bounty Program

ArduPilot participates in responsible disclosure programs. While we don't currently offer monetary rewards, we acknowledge security researchers who help improve ArduPilot's security through:

- Public acknowledgment (if desired)
- CVE credit
- Recognition in release notes
- Hall of Fame listing

## Contact

- **Security Team**: security@ardupilot.org
- **General Support**: https://discuss.ardupilot.org
- **Development**: https://discord.gg/ardupilot

## Legal

We are committed to working with security researchers under the principle of "good faith" and will not pursue legal action against researchers who:

- Make a good faith effort to avoid privacy violations, data destruction, and service interruption
- Only interact with accounts you own or with explicit permission
- Do not exploit vulnerabilities beyond the minimum necessary to demonstrate the issue
- Report vulnerabilities promptly
- Keep details confidential until coordinated disclosure

---

*This security policy is effective as of 2025 and will be reviewed annually.*
