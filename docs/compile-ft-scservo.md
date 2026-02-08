### Tutorial: Compiling FT_SCServo_Debug_Qt on macOS

#### 1. Install System Dependencies
Use Homebrew to install the Qt5 framework and essential build tools:
```bash
brew install qt@5 cmake git gcc
```

#### 2. Configure Environment Paths
Add Qt5 to your system path so `qmake` and the compiler can locate the libraries (adjust path if using Intel Mac):
```bash
export PATH="/opt/homebrew/opt/qt@5/bin:$PATH"
export LDFLAGS="-L/opt/homebrew/opt/qt@5/lib"
export CPPFLAGS="-I/opt/homebrew/opt/qt@5/include"
```

#### 3. Clone and Build the Project
Download the source code and use `qmake` to generate the Makefile specifically for your architecture, followed by `make` to compile:
```bash
git clone [https://github.com/Kotakku/FT_SCServo_Debug_Qt.git](https://github.com/Kotakku/FT_SCServo_Debug_Qt.git)
cd FT_SCServo_Debug_Qt/
qmake .
make
```

#### 4. Launch the Application
On macOS, the build process creates a `.app` bundle. Launch it from the terminal:
```bash
open FT_SCServo_Debug_Qt.app
```