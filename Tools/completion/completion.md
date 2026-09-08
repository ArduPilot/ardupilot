# Shell Completion

## on Bash

To install the completion, you should either, source the main script on your current terminal or source the main script on your bash configuration.

### Direct use

From ArduPilot root directory, simply use :

```bash
source ./Tools/completion/completion.bash
```

And now completion works from your terminal instance. If you close the terminal, the completion feature is removed.

### Permanent use

Edit you `.bashrc` file, it is on your Home directory but it is a hidden file (CTRL+H on Ubuntu to reveal them). Then put at the end of the file :

```bash
source PATH_TO_ARDUPILOT_DIRECTORY/Tools/completion/completion.bash
```

where PATH_TO_ARDUPILOT_DIRECTORY is the path to ArduPilot directory.

### Usage

You can now abuse of your TAB key on `waf` and `sim_vehicle.py` call ! See the video at the end.

Note: completion is registered for the `waf` command. Since ArduPilot's
`waf` is called as `./waf` from the root directory, add the following alias
to your `.bashrc` so you can call it as `waf` and get completion:

```bash
alias waf="./waf"
```

After adding the alias, use `waf` (without `./`) to benefit from TAB completion.

## On ZSH

Zsh don't allow live loading of completion. So you have to source the completion script in your `.zshrc` file. Like for Bash, you will find it hiding on your home !
Put at the end of the file :

```bash
source PATH_TO_ARDUPILOT_DIRECTORY/Tools/completion/completion.zsh
```

where PATH_TO_ARDUPILOT_DIRECTORY is the path to ArduPilot directory. Notice the difference, the extension is `.zsh` !
