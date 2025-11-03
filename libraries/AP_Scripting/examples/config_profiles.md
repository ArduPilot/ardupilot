# Configuration Profiles LUA script

This script allows different configuration profiles to be selected by changing single parameters created by the script.

You may use this script to allow users to rapidly reconfigure the vehicle based on the role the vehicle will be used in.

## Terms

### Domains

Parameters are divided into domains; a parameter can only exist in a single domain.  So you may have a domain for different battery configurations and a domain for different payload configurations.

Each domain specifies what parameters are relevant in an all_param_defaults attribute.

### Profiles
Each domain has a number of profiles which can be switched between via setting a parameter.  The profiles specify parameter values which override the defaults specified in the domain's all_param_defaults attribute.

### Modes

Each domain can be in one of three modes:
 - do nothing
 - use the default parameter values only
 - use parameters from the selected profile, taking the parameter value from the defaults stored in the domain if the profile does not specify a value

### Use

Each domain has a corresponding "_SEL" parameter associated.  So, for example, a "PAY" domain will have "CFG_PAY_SEL" as its parameter.

If the value of this parameter is "-1" then the script will take no actions for this domain.  This is useful if you want to modify the parameters by hand.

If the value of this parameter is "0" then the defaults for the domain will be applied.  No default parameter can be marked as "must be set in the profile" (as setting a subset of the domain parameters may be dangerous).

Any other value of the "_SEL" parameter is considered to be a reference to a specific profile - which must exist in the "profiles" section in the domain.

### Notes

Any time a parameter is changed by the script a reboot is required.
