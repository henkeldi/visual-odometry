
# Visual Odometry

*C++ Implementation of Visual Odometry*

## Prerequisites

<details><summary>OpenCV</summary>

```bash
VERSION=4.1.2
mkdir -p $HOME/src
cd $HOME/src
wget https://github.com/opencv/opencv/archive/$VERSION.tar.gz
tar xvf $VERSION.tar.gz
rm $VERSION.tar.gz
wget https://github.com/opencv/opencv_contrib/archive/$VERSION.tar.gz
tar xvf $VERSION.tar.gz
rm $VERSION.tar.gz
cd ./opencv-$VERSION
mkdir build
cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=$HOME/src/opencv_contrib-$VERSION/modules\
      -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF\
      -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DENABLE_PRECOMPILED_HEADERS=OFF ..
make -j8
sudo make install
```

</details>

## Build

```bash
cmake .
make
```

## Run

```bash
./main
```

## References

* [Twitchslam by George Hotz](https://github.com/geohot/twitchslam)
* [VINS Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
