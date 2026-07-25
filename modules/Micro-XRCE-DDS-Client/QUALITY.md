This document is a declaration of software quality for the **eProsima Micro XRCE-DDS Client** based on the guidelines provided in the [ROS 2 REP-2004 document](https://www.ros.org/reps/rep-2004.html).

# Quality Declaration

**eProsima Micro XRCE-DDS Client** is a C99 library that provides a DDS-XRCE implementation for using DDS in eXtreme Resources Constrained Devices. It follows the [DDS-XRCE standard](https://www.omg.org/spec/DDS-XRCE/About-DDS-XRCE/).

**eProsima Micro XRCE-DDS Client** claims to be in the **Quality Level 1** category.

Below are the rationales, notes and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 1 in REP-2004](https://www.ros.org/reps/rep-2004.html#package-requirements).

## Version Policy [1]

### Version Scheme [1.i]

The **Versioning Policy Declaration** for **eProsima Micro XRCE-DDS Client** can be found [here](VERSIONING.md) and it adheres to [`semver`](https://semver.org/).

### Version Stability [1.ii]

**eProsima Micro XRCE-DDS Client** is at a stable version, i.e. `>=1.0.0`.
The latest version and the release notes can be found [here](https://github.com/eProsima/Micro-XRCE-DDS-Client/releases).

### Public API Declaration [1.iii]

The public API is documented [in oficial documentation Read the Docs](https://micro-xrce-dds.docs.eprosima.com/en/latest/client.html#api).

### API Stability Policy [1.iv]

**eProsima Micro XRCE-DDS Client** will only break public API between major releases.

### ABI Stability Policy [1.v]

Any ABI break in **eProsima Micro XRCE-DDS Client** will be done between minor versions and it should be clearly stated in the release notes, note that minor releases can happen within a ROS distribution.


## Change Control Process [2]

The stability of **eProsima Micro XRCE-DDS Client** is ensured through reviews, CI and tests.
The change control process can be found in [CONTRIBUTING](CONTRIBUTING.md)

All changes to **eProsima Micro XRCE-DDS Client** occur through pull requests that are required to pass all CI tests.
In case of failure, only maintainers can merge the pull request, and only when there is enough evidence that the failure is unrelated to the change.
Additionally, all pull requests must have at least one positive review from another contributor that did not author the pull request.

### Change Requests [2.i]

All changes will occur through a pull request.

### Contributor Origin [2.ii]

**eProsima Micro XRCE-DDS Client** uses the [Developer Certificate of Origin (DCO)](https://developercertificate.org/) as its confirmation of contributor origin policy since version 2.0.0.
More information can be found in [CONTRIBUTING](CONTRIBUTING.md)

### Peer Review Policy [2.iii]

All pull requests will be peer-reviewed by at least one other contributor who did not author the pull request. Approval is required before merging.

### Continuous Integration [2.iv]

All pull requests must pass CI to be considered for merging, unless maintainers consider that there is enough evidence that the failure is unrelated to the changes.
CI testing is automatically triggered by incoming pull requests.
Current nightly results for all supported platforms can be checked at the links:

* Linux [![Linux ci](http://jenkins.eprosima.com:8080/view/Nightly/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/badge/icon?subject=%20%20%20Linux%20CI%20)](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/)
* Windows [![Windows ci](http://jenkins.eprosima.com:8080/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Windows/badge/icon?subject=%20%20%20%20Windows%20CI%20)](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Windows/)

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging as stated in [CONTRIBUTING](CONTRIBUTING.md).

## Documentation [3]

### Feature Documentation [3.i]

**eProsima Micro XRCE-DDS Client** has a documented features list hosted by [Read the Docs](https://micro-xrce-dds.docs.eprosima.com/en/latest/client.html).

### Public API Documentation [3.ii]

**eProsima Micro XRCE-DDS Client** includes a complete API Reference which is hosted in [Read the Docs](https://micro-xrce-dds.docs.eprosima.com/en/latest/client.html#api).

### License [3.iii]

The license for **eProsima Micro XRCE-DDS Client** is Apache 2.0, and a summary can be found in each source file.
A full copy of the license can be found [here](LICENSE).

### Copyright Statements [3.iv]

The **eProsima Micro XRCE-DDS Client** copyright holder provides a statement of copyright in each source file.

## Testing [4]

### Feature Testing [4.i]

**eProsima Micro XRCE-DDS Client** provides tests which simulate typical usage, and they are located in the [`test` directory](test).
New features are required to have tests before being added as stated in [CONTRIBUTING](CONTRIBUTING.md).
Current nightly results can be found here:

* Linux [![Linux ci](http://jenkins.eprosima.com:8080/view/Nightly/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/badge/icon?subject=%20%20%20Linux%20CI%20)](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/)
* Windows [![Windows ci](http://jenkins.eprosima.com:8080/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Windows/badge/icon?subject=%20%20%20%20Windows%20CI%20)](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Windows/)
### Public API Testing [4.ii]

Each part of the public API has tests, and new additions or changes to the public API require tests before being added.
The tests aim to cover typical usage and corner cases.

### Coverage [4.iii]

[![Coverage](https://img.shields.io/jenkins/coverage/cobertura?jobUrl=http%3A%2F%2Fjenkins.eprosima.com%3A8080%2Fview%2FMicro%2520XRCE%2Fjob%2FMicro-XRCE-DDS-Client%2520Nightly%2520Master%2520Linux)](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/coverage/)

*eProsima Micro XRCE-DDS Client* aims to provide a line coverage **above 90%**.
*Micro XRCE-DDS Client* code coverage policy comprises:
1. All contributions to *Micro XRCE-DDS Client* must increase (or at least keep) the current line coverage.
   This is done to ensure that the **90%** line coverage goal is eventually met.
2. Line coverage regressions are only permitted if properly justified and accepted by maintainers.
3. If the CI system reports a coverage regression after a pull request has been merged, the maintainers must study the case and decide how to proceed, most often reverting the changes and asking for a more thorough testing of the committed changes.
4. This policy is enforced through the [nightly Micro XRCE-DDS Client CI job](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/).

As stated in [CONTRIBUTING.md](CONTRIBUTING.md), developers and contributors are required to run a line coverage assessment locally before submitting a PR.

### Performance [4.iv]

**eProsima Micro XRCE-DDS Client** does provide performance tests regarding memory consumption due to the nature of the library.
These memory consuption tests are under `test/memory` and evaluates the static size (`text`, `bss` and `data` sections) of the library built with different configuration profiles.
Also, a `stack` consumption analysis is provided by using [Valgrind Massif tool](https://valgrind.org/docs/manual/ms-manual.html).

All these results are available in [Micro-XRCE-DDS-Client Nightly Master Performance job](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Performance/plot/)

Any performance regression detected in *eProsima Micro XRCE-DDS Client* would be analysed and, in case it is related to **eProsima Micro XRCE-DDS Client** or could be solved by modifying this library, changes can be made to the library in order to revert the performance regression.

### Linters and Static Analysis [4.v]

**eProsima Micro XRCE-DDS Client** [code style](https://github.com/eProsima/cpp-style) is enforced using [*uncrustify*](https://github.com/uncrustify/uncrustify).
Among the CI tests, there are tests that ensure that every pull request is compliant with the code style.
The latest pull request results can be seen [here](http://jenkins.eprosima.com:8080/job/Micro-XRCE-DDS-Client%20Manual%20Linux/lastBuild/).

**eProsima Micro XRCE-DDS Client** uses [Synopsis Coverity static code analysis](https://scan.coverity.com/projects/eprosima-micro-xrce-dds-client).

**eProsima Micro XRCE-DDS Client** uses [CodeQL](https://github.com/eProsima/Micro-XRCE-DDS-Client/security/code-scanning?query=tool%3ACodeQL) to find security issues on the code.

## Dependencies [5]

### Direct Runtime Dependencies [5.iii]

**eProsima Micro XRCE-DDS Client**  depends on the following packages:
- `eProsima Micro CDR`

**eProsima Micro CDR** Quality Declaration can be found [here](https://github.com/eProsima/Micro-CDR/blob/master/QUALITY.md). Currently, **eProsima Micro CDR** claims to be in the **Quality Level 1** category.

## Platform Support [6]

**eProsima Micro XRCE-DDS Client** supports the following platforms and tests each change against all of them as can be seen in the current nightly results:

* Linux [![Linux ci](http://jenkins.eprosima.com:8080/view/Nightly/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/badge/icon?subject=%20%20%20Linux%20CI%20)](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Linux/)
* Windows [![Windows ci](http://jenkins.eprosima.com:8080/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Windows/badge/icon?subject=%20%20%20%20Windows%20CI%20)](http://jenkins.eprosima.com:8080/view/Micro%20XRCE/job/Micro-XRCE-DDS-Client%20Nightly%20Master%20Windows/)

More information about the supported platforms can be found in [PLATFORM_SUPPORT](PLATFORM_SUPPORT.md)

## Vulnerability Disclosure Policy [7.i]

**eprosima Micro XRCE-DDS Client** vulnerability Disclosure Policy can be found [here](https://github.com/eProsima/policies/blob/main/VULNERABILITY.md)

# Current Status Summary

The chart below compares the requirements in the [REP-2004](https://www.ros.org/reps/rep-2004.html#quality-level-comparison-chart) with the current state of **eprosima Micro XRCE-DDS Client**
| Number  | Requirement                                       | Current State |
| ------- | ------------------------------------------------- | ------------- |
| 1       | **Version policy**                                | ---           |
| 1.i     | Version Policy available                          | ✓             |
| 1.ii    | Stable version                                    | ✓             |
| 1.iii   | Declared public API                               | ✓             |
| 1.iv    | API stability policy                              | ✓             |
| 1.v     | ABI stability policy                              | ✓             |
| 2       | **Change control process**                        | ---           |
| 2.i     | All changes occur on change request               | ✓             |
| 2.ii    | Contributor origin (DCO, CLA, etc)                | ✓             |
| 2.iii   | Peer review policy                                | ✓             |
| 2.iv    | CI policy for change requests                     | ✓             |
| 2.v     | Documentation policy for change requests          | ✓             |
| 3       | **Documentation**                                 | ---           |
| 3.i     | Per feature documentation                         | ✓             |
| 3.ii    | Per public API item documentation                 | ✓             |
| 3.iii   | Declared License(s)                               | ✓             |
| 3.iv    | Copyright in source files                         | ✓             |
| 3.v.a   | Quality declaration linked to README              | ✓             |
| 3.v.b   | Centralized declaration available for peer review | ✓             |
| 4       | **Testing**                                       | ---           |
| 4.i     | Feature items tests                               | ✓             |
| 4.ii    | Public API tests                                  | ✓             |
| 4.iii.a | Using coverage                                    | ✓             |
| 4.iii.b | Coverage policy                                   | ✓             |
| 4.iv.a  | Performance tests (if applicable)                 | ✓             |
| 4.iv.b  | Performance tests policy                          | ✓             |
| 4.v.a   | Code style enforcement (linters)                  | ✓             |
| 4.v.b   | Use of static analysis tools                      | ✓             |
| 5       | **Dependencies**                                  | ---           |
| 5.iii   | Justifies quality use of dependencies             | ✓             |
| 6       | **Platform support**                              | ---           |
| 6.i     | Support targets Tier1 ROS platforms               | ✓             |
| 7       | **Security**                                      | ---           |
| 7.i     | Vulnerability Disclosure Policy                   | ✓             |
