# Quick Pyenv Tutorial

`pyenv` lets you easily switch between multiple versions of Python.

## 1. List Available Versions
Check what Python versions you can install:
```bash
pyenv install --list | grep 3.11  # Filter for specific versions
```

## 2. Install a Python Version
Install a specific version:
```bash
pyenv install 3.11.7
```

## 3. Set Python Version
### Local (Per-project)
Sets `.python-version` in the current directory. Automatically activates when you enter the folder.
```bash
pyenv local 3.11.7
```

### Global (Default)
Sets the default Python version for your user.
```bash
pyenv global 3.11.7
```

### Shell (Current Session Only)
```bash
pyenv shell 3.11.7
```

## 4. Check Active Version
See which version is currently active and why.
```bash
pyenv version
# Output: 3.11.7 (set by /path/to/.python-version)
```

## 5. Virtual Environments (with `pyenv-virtualenv`)
Create a virtualenv assigned to a specific Python version.

**Create:**
```bash
pyenv virtualenv 3.11.7 my-env-name
```

**Activate Manually:**
```bash
pyenv activate my-env-name
```
*(Note: If you set `pyenv local my-env-name`, it activates automatically).*

**Delete:**
```bash
pyenv uninstall my-env-name
```
