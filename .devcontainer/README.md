# ArduPilot Dev Container

This directory contains the VS Code Dev Container configuration for ArduPilot development.

## What is a Dev Container?

A development container is a Docker container configured as a full-featured development environment. It provides:

- ✅ **Consistent Environment**: Same tools and dependencies for all developers
- ✅ **Quick Setup**: Get started in minutes instead of hours
- ✅ **Isolated**: No conflicts with your host system
- ✅ **Pre-configured**: VS Code extensions and settings ready to go
- ✅ **Portable**: Works on Windows, macOS, and Linux

## Prerequisites

1. **VS Code**: Download from [code.visualstudio.com](https://code.visualstudio.com/)
2. **Docker**: Install [Docker Desktop](https://www.docker.com/products/docker-desktop)
3. **Dev Containers Extension**: Install from [VS Code Marketplace](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Quick Start

1. Open ArduPilot repository in VS Code:
   ```bash
   cd ardupilot
   code .
   ```

2. When prompted, click **"Reopen in Container"**
   
   Or manually: Press `F1` → `Dev Containers: Reopen in Container`

3. Wait for the container to build (first time: ~10 minutes)

4. Start developing! Try:
   ```bash
   ./waf configure --board sitl
   ./waf copter
   ./Tools/autotest/sim_vehicle.py -v ArduCopter
   ```

## What's Included

### Pre-installed Tools

- **Build System**: WAF, Make, CMake
- **Compilers**: GCC, G++, ARM toolchains
- **Python**: Python 3 with development packages
- **Debugging**: GDB, Valgrind
- **Version Control**: Git, Git LFS
- **Analysis**: ccache for faster builds

### VS Code Extensions

Automatically installed:

- C/C++ IntelliSense
- Python with Pylance
- GitLens
- CMake Tools
- Shell formatting
- YAML support
- Spell checker

### Port Forwarding

The following ports are automatically forwarded:

- **5760**: MAVLink SITL primary
- **5761**: MAVLink Console
- **5762-5763**: Additional MAVLink ports

Access MAVProxy or Mission Planner on your host machine using `localhost:5760`.

## Features

### Persistent Cache

Build cache is preserved in a Docker volume for faster rebuilds:

```bash
# Check cache stats
ccache -s

# Clear if needed
ccache -C
```

### Pre-configured Settings

The container includes optimized VS Code settings:

- C++17 standard
- Python linting with flake8
- Auto-formatting on save
- Git integration
- 120-character line rulers

### Development Features

Additional dev container features:

- **Oh My Zsh**: Enhanced shell experience
- **GitHub CLI**: Manage PRs and issues from terminal
- **Common utilities**: wget, curl, nano, vim

## Customization

### Modify Container Configuration

Edit `.devcontainer/devcontainer.json` to:

- Add VS Code extensions
- Change port forwarding
- Modify container environment variables
- Add custom settings

### Rebuild Container

After modifying `devcontainer.json` or `Dockerfile`:

1. Press `F1`
2. Select: `Dev Containers: Rebuild Container`

## Troubleshooting

### Container Fails to Start

```bash
# Check Docker is running
docker --version

# View Docker logs
docker logs <container-id>

# Rebuild from scratch
# In VS Code: F1 → Dev Containers: Rebuild Container Without Cache
```

### Permission Issues

If you encounter permission errors:

```bash
# Inside container
sudo chown -R ardupilot:ardupilot /ardupilot
```

### Slow Build Times

First build is slow (~10 minutes). Subsequent builds use cache and are much faster (~30 seconds).

To speed up:
- Ensure Docker Desktop has sufficient CPU/RAM allocated
- Check that ccache volume is properly mounted

### Port Already in Use

If port 5760 is already in use:

1. Edit `.devcontainer/devcontainer.json`
2. Change the `forwardPorts` array
3. Rebuild container

## Advanced Usage

### Use with Docker Compose

The dev container can also work with the project's `docker-compose.yml`:

```bash
# Start services
docker-compose up -d ardupilot-dev

# Attach VS Code to running container
# F1 → Dev Containers: Attach to Running Container
```

### Multiple Workspaces

You can have multiple ArduPilot instances in separate containers:

```bash
# Clone to different directory
git clone https://github.com/ArduPilot/ardupilot.git ardupilot-experimental

# Open in new VS Code window
code ardupilot-experimental

# Each gets its own container
```

### Custom Dockerfile

To use a custom Dockerfile:

1. Copy `Dockerfile` to `.devcontainer/Dockerfile`
2. Update `devcontainer.json`: `"dockerFile": "Dockerfile"`
3. Make your changes
4. Rebuild container

## Performance Tips

- **Allocate Resources**: Give Docker Desktop at least 4GB RAM and 2 CPUs
- **Use Volumes**: Mounted volumes (ccache, vscode-server) improve performance
- **SSD Storage**: Docker images on SSD are significantly faster
- **Close Unused Containers**: `docker container prune` to free resources

## Security

The dev container runs as a non-root user (`ardupilot`) for security. To run commands as root:

```bash
sudo <command>
```

Password is not required due to NOPASSWD sudoers configuration (safe for development containers).

## Support

- **Dev Container Issues**: [VS Code Dev Containers Documentation](https://code.visualstudio.com/docs/devcontainers/containers)
- **ArduPilot Development**: [ArduPilot Discord #development](https://ardupilot.org/discord)
- **General Help**: See [DEVELOPMENT.md](../DEVELOPMENT.md)

## Alternative: GitHub Codespaces

This dev container configuration also works with GitHub Codespaces:

1. Go to the ArduPilot repository on GitHub
2. Click **Code** → **Codespaces** → **Create codespace on master**
3. Wait for environment to load
4. Start developing in the browser or connect with VS Code

---

**Happy Coding!** 🚁✨
