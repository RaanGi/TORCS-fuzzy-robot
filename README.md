# TORCS-fuzzy-robot
This is a [TORCS](http://torcs.sourceforge.net/) robot that make use of fuzzy logic thanks to [fuzzylite](https://github.com/fuzzylite/fuzzylite).
This project only works on LINUX.

## Installation

First of all, you need to download and install TORCS, and also download and build Fuzzylite source code.

Then, go to drivers folder.

```bash
cd $TORCS_BASE
cd src/drivers
```

Paste the RanGi folder there. 

At this point, fuzzylite source should has been built.

Inside RanGi folder, paste fuzzylite folder. This contains fuzzylite source and the release folder.

Now is time to make an install our robot. To do this, there's a bash file that will make our live easier.

```bash
cd RanGi
chmod 0755 make.sh
./make.sh
```

And now our robot is ready to race.

## Usage

Go to the robot folder and execute fuzzyengines file.

```bash
cd $TORCS_BASE
cd src/drivers/RanGi
./fuzzyengines
```

After that run torcs and that's all.

```bash
torcs
```
