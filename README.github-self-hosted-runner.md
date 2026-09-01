# Running ArduPilot CI on your own (self-hosted) GitHub runners

The workflows in this tree select their runner through two GitHub
Actions *repository variables*:

```yaml
runs-on: ${{ vars.CI_RUNNER || 'ubuntu-22.04' }}        # linux jobs
runs-on: ${{ vars.CI_WIN_RUNNER || 'windows-latest' }}  # cygwin build
```

If the variables are **not defined** (the default everywhere, including
the upstream ArduPilot repository) every job resolves to exactly the
original GitHub-hosted label and nothing changes. If you define them in
your fork, your own pushes run on your own hardware instead - useful
when the hosted queue is slow, or when your machine is simply faster.

## Quick start (fork)

1. Register a runner on your fork: *Settings -> Actions -> Runners ->
   New self-hosted runner*, and give it a custom label, e.g. `mybox`.
2. Define the variables: *Settings -> Secrets and variables -> Actions
   -> Variables*:
   * `CI_RUNNER` = `mybox` (routes Linux jobs)
   * `CI_WIN_RUNNER` = `Windows` (routes the Cygwin build to a runner
     carrying the `Windows` label)
3. Push. Jobs dispatch to your runners; delete the variables to return
   to GitHub-hosted at any time. No workflow edits either way.

Do **not** give a self-hosted runner a GitHub-hosted image label such
as `ubuntu-22.04` or `windows-latest`: hosted runners take precedence
for those labels, so your runner will starve, and when it does catch a
job you can no longer tell which environment produced a result.

## Linux runner in Docker

### Installing Docker first

Any recent Docker works. On Debian/Ubuntu the quickest path:

```bash
curl -fsSL https://get.docker.com | sh          # docker engine + cli
sudo usermod -aG docker $USER                   # then log out/in once
```

or, to run the daemon **rootless** (no root daemon on the machine -
recommended for a runner box; this is also what changes the socket
path mentioned below):

```bash
sudo apt-get install -y docker-ce-rootless-extras uidmap
dockerd-rootless-setuptool.sh install
systemctl --user enable --now docker
echo "export DOCKER_HOST=unix:///run/user/$(id -u)/docker.sock" >> ~/.bashrc
```

Check it works before going further: `docker run --rm hello-world`.

### Watching your runner

Once the runner container (below) is up, its console - registration,
"Listening for Jobs", every job start/finish - is just its container
log:

```bash
docker logs gh-runner-mybox --tail 20 -f        # live tail
docker ps --filter name=gh-runner               # is it up
gh api repos/<you>/ardupilot/actions/runners \
   -q '.runners[] | "\(.name) \(.status) busy=\(.busy)"'   # GitHub's view
```

### The runner container

A containerised runner works well; ArduPilot jobs run inside
`ardupilot/ardupilot-dev-*` job containers, so the runner needs access
to a Docker daemon:

```bash
TOKEN=$(gh api -X POST repos/<you>/ardupilot/actions/runners/registration-token -q .token)
docker run -d --restart always --name gh-runner \
  -e REPO_URL=https://github.com/<you>/ardupilot \
  -e RUNNER_TOKEN=$TOKEN -e RUNNER_NAME=mybox \
  -e LABELS=self-hosted,mybox \
  -e RUNNER_WORKDIR=/tmp/gh-runner-work \
  -e ACTIONS_RUNNER_HOOK_JOB_STARTED=/tmp/gh-runner-work/prejob.sh \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v /tmp/gh-runner-work:/tmp/gh-runner-work \
  --cpus 4 --memory 6g \
  myoung34/github-runner:ubuntu-jammy
```

Hard-won details, all one-time:

* **Rootless Docker**: mount the rootless socket
  (`/run/user/<uid>/docker.sock`) as `/var/run/docker.sock` instead -
  the rootful socket appears as `nobody:nogroup` inside the container
  and is unusable even by root.
* **Mount the workdir at the identical path** on host and in the
  container (as above). The runner passes that path to the daemon when
  starting job containers, and the daemon resolves it on the *host*.
* **`/actions-runner/externals` and `/opt/hostedtoolcache`** must exist
  on the host (the daemon mounts them into job containers from host
  paths): copy `externals` out of the runner image once
  (`docker cp gh-runner:/actions-runner/externals /actions-runner/externals`)
  and `mkdir` a user-owned `/opt/hostedtoolcache`.
* **Persistent-workspace poisoning**: some workflows use sparse
  checkouts, and sparse state survives in `.git` between jobs on a
  reused workspace, leaving later full checkouts nearly empty. Use a
  job-started hook (`prejob.sh`) that removes
  `.git/info/sparse-checkout` / sets `core.sparseCheckout=false` in any
  workspace repo, and `chmod -R a+rwX` the `_temp` dir so job
  containers running as a different uid can write runner state files.
* Replace runner containers only when idle: killing one mid-job leaves
  GitHub holding the runner "busy" with an orphaned job for ~10
  minutes, blocking dispatch to the replacement.

## Windows runner (for the Cygwin build)

Install the runner natively (GitHub Actions cannot run Windows job
containers; the Cygwin workflow's steps run directly on the host). Any
Windows 10/11 or Server VM with a few cores, ~8GB RAM and ~40GB free
disk is enough.

### Registering the VM as a runner (PowerShell)

Grab a registration token from your fork - either *Settings -> Actions
-> Runners -> New self-hosted runner* (it shows the token inline), or
from any machine with `gh`:

```bash
gh api -X POST repos/<you>/ardupilot/actions/runners/registration-token -q .token
```

Then on the VM, in PowerShell:

```powershell
mkdir C:ctions-runner ; cd C:ctions-runner
Invoke-WebRequest -Uri https://github.com/actions/runner/releases/latest/download/actions-runner-win-x64.zip -OutFile runner.zip
Expand-Archive runner.zip -DestinationPath .
./config.cmd --url https://github.com/<you>/ardupilot --token <TOKEN>
#   runner group: Enter (Default)
#   name:         e.g. buzz-windows
#   extra labels: Enter (the automatic self-hosted,Windows,X64 are enough;
#                 do NOT add windows-latest - see the label warning above)
#   work folder:  Enter (_work)
./run.cmd            # interactive: shows "Listening for Jobs" and every job live
```

(If the `latest/download` URL 404s, take the exact
`actions-runner-win-x64-<version>.zip` asset URL from
https://github.com/actions/runner/releases.)

`./run.cmd` runs in the foreground, which is ideal while setting up -
it is the Windows equivalent of `docker logs -f`. When you are happy
with it, install it as a service instead so it survives logouts and
reboots (admin PowerShell):

```powershell
./svc.cmd install ; ./svc.cmd start
```

Finally set the fork variable `CI_WIN_RUNNER` = `Windows` and the
Cygwin build dispatches to the VM on your next push.

### One-time VM preparation

A bare machine needs, once:

* **Git for Windows** on PATH (checkout falls back to a REST download
  that cannot fetch submodules without it), plus
  `git config --system core.longpaths true`.
* **PowerShell execution policy**:
  `Set-ExecutionPolicy RemoteSigned -Scope LocalMachine` (actions
  generate .ps1 scripts; the OS default blocks them).
* **Firewall rules** for the built SITL binaries if you dislike
  popups - all test traffic is loopback, which Windows permits
  regardless, so the prompts are cosmetic.
* Expect the first build to be slow (Cygwin install + cold compile);
  the workspace, Cygwin and ccache persist, so later runs are much
  faster than GitHub-hosted.

## Security

Self-hosted runners on a **public** fork execute whatever a workflow
tells them to. Keep *Settings -> Actions -> "Require approval for all
outside collaborators"* strict, prefer ephemeral runners where
practical, and treat a runner with a mounted Docker socket as
root-equivalent on its host.
