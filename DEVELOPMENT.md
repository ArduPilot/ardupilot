# ArduPilot Development Guide

## Quick Start

### Option 1: VS Code Dev Container (Recommended)

The fastest way to get started with ArduPilot development:

1. **Prerequisites**:
   - [VS Code](https://code.visualstudio.com/)
   - [Docker Desktop](https://www.docker.com/products/docker-desktop)
   - [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

2. **Open in Container**:
   ```bash
   # Clone the repository
   git clone --recursive https://github.com/ArduPilot/ardupilot.git
   cd ardupilot
   
   # Open in VS Code
   code .
   ```

3. **Reopen in Container**:
   - Press `F1` or `Ctrl+Shift+P`
   - Select: `Dev Containers: Reopen in Container`
   - Wait for the container to build (first time only, ~10 minutes)

4. **Build and Run**:
   ```bash
   # Configure for SITL
   ./waf configure --board sitl
   
   # Build ArduCopter
   ./waf copter
   
   # Run SITL simulation
   ./Tools/autotest/sim_vehicle.py -v ArduCopter
   ```

### Option 2: Docker Compose

For multi-service development or running analysis tools:

1. **Start Development Environment**:
   ```bash
   # Main development container
   docker-compose up -d ardupilot-dev
   docker-compose exec ardupilot-dev bash
   ```

2. **Run SITL with Graphics**:
   ```bash
   # Enable GUI support (Linux with X11)
   export DISPLAY=:0
   docker-compose --profile simulation up ardupilot-sitl
   ```

3. **Start Code Analysis Tools**:
   ```bash
   # Start SonarQube for code quality analysis
   docker-compose --profile analysis up -d sonarqube
   # Access at http://localhost:9090
   ```

4. **Build Documentation**:
   ```bash
   # Generate and serve documentation
   docker-compose --profile docs up ardupilot-docs
   # Access at http://localhost:8080
   ```

### Option 3: Manual Setup

Follow the traditional setup process:

```bash
# Install prerequisites
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Configure and build
./waf configure --board sitl
./waf copter
```

See [BUILD.md](BUILD.md) for detailed build instructions.

## Development Workflow

### Building for Different Boards

```bash
# List available boards
./waf list_boards

# Configure for specific board
./waf configure --board CubeBlack
./waf copter

# Build all vehicle types
./waf copter plane rover sub
```

### Running Tests

```bash
# Run unit tests
./waf check

# Run specific test suite
./waf --target tests/test_math

# Run SITL autotest
Tools/autotest/autotest.py build.Copter test.Copter
```

### Code Quality

```bash
# Format code (Python)
black Tools/

# Lint Python code
flake8 Tools/

# Check for common issues
pre-commit run --all-files
```

### Debugging

#### VS Code Debugging

The dev container includes pre-configured launch configurations:

1. Press `F5` to start debugging
2. Select configuration: `Debug SITL`
3. Set breakpoints in your code
4. Use debug console to inspect variables

#### GDB Command Line

```bash
# Build with debug symbols
./waf configure --board sitl --debug
./waf copter

# Run under GDB
gdb build/sitl/bin/arducopter
(gdb) run --model quad
```

## Container Management

### Useful Docker Commands

```bash
# View running containers
docker-compose ps

# View container logs
docker-compose logs -f ardupilot-dev

# Restart container
docker-compose restart ardupilot-dev

# Remove all containers and volumes
docker-compose down -v

# Rebuild container (after Dockerfile changes)
docker-compose build --no-cache ardupilot-dev
```

### Persist Build Cache

The ccache is mounted as a volume to speed up rebuilds:

```bash
# Check cache stats
ccache -s

# Clear cache if needed
ccache -C
```

## IDE Configuration

### VS Code Extensions

The dev container automatically installs:

- C/C++ IntelliSense
- Python support with Pylance
- GitLens for enhanced Git integration
- CMake Tools
- Code spell checker

### Custom Settings

Add workspace-specific settings in `.vscode/settings.json`:

```json
{
  "C_Cpp.default.compileCommands": "${workspaceFolder}/build/sitl/compile_commands.json",
  "python.analysis.extraPaths": ["${workspaceFolder}/Tools"]
}
```

## Advanced Development

### Custom Build Options

```bash
# Enable specific features
./waf configure --board sitl \
    --enable-scripting \
    --enable-networking-tests \
    --sitl-osd

# Build with coverage
./waf configure --board sitl --coverage
./waf copter
Tools/autotest/autotest.py build.Copter test.Copter
# Coverage report in build/sitl/coverage/
```

### Profiling

```bash
# Build with profiling
./waf configure --board sitl --debug
./waf copter

# Run with gprof
./build/sitl/bin/arducopter --model quad
gprof build/sitl/bin/arducopter gmon.out > analysis.txt
```

### Memory Analysis

```bash
# Run with Valgrind
valgrind --leak-check=full \
    --track-origins=yes \
    ./build/sitl/bin/arducopter --model quad
```

## Code Analysis

### Running Static Analysis

```bash
# CodeQL analysis
codeql database create codeql-db --language=cpp
codeql database analyze codeql-db --format=sarif-latest --output=results.sarif

# Clang-tidy
clang-tidy libraries/AP_Math/*.cpp -- -I libraries/
```

### SonarQube Analysis

```bash
# Start SonarQube (via Docker Compose)
docker-compose --profile analysis up -d sonarqube

# Wait for SonarQube to start (~2 minutes)
# Access at http://localhost:9090

# Run analysis
build-wrapper-linux-x86-64 --out-dir bw-output ./waf build
sonar-scanner
```

## Troubleshooting

### Common Issues

**Container won't start:**
```bash
# Check Docker is running
docker info

# Reset Docker environment
docker-compose down -v
docker-compose up --build
```

**Build failures:**
```bash
# Clean build artifacts
./waf clean
./waf distclean

# Update submodules
git submodule update --init --recursive
```

**Permission issues:**
```bash
# Fix ownership (inside container)
sudo chown -R ardupilot:ardupilot /ardupilot
```

## Contributing

Before submitting a PR:

1. ✅ Run tests: `./waf check`
2. ✅ Format code: `pre-commit run --all-files`
3. ✅ Update documentation
4. ✅ Add entry to release notes (if applicable)
5. ✅ Ensure CI passes

See [CONTRIBUTING.md](.github/CONTRIBUTING.md) for detailed guidelines.

## Resources

- 📖 [Developer Wiki](https://ardupilot.org/dev/)
- 💬 [Discord Server](https://ardupilot.org/discord)
- 🐛 [Issue Tracker](https://github.com/ArduPilot/ardupilot/issues)
- 📧 [Mailing List](https://discuss.ardupilot.org)
- 🔒 [Security Policy](SECURITY.md)

## Getting Help

- **General Questions**: [Discord #general](https://ardupilot.org/discord)
- **Development Discussion**: [Discourse Development](https://discuss.ardupilot.org/c/development-team)
- **Bug Reports**: [GitHub Issues](https://github.com/ArduPilot/ardupilot/issues)
- **Security Issues**: security@ardupilot.org

---

**Happy Coding!** 🚁✨
