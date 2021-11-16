# elmo driver

### describe

Elmo driver based on [soem v1.4.0](https://github.com/OpenEtherCATsociety/SOEM/tree/v1.4.0)

### download

```
git clone https://www.lejuhub.com/fuyou/ethercat_elmo.git
```

### build

```
cd ethercat_elmo/
mkdir build && cd build
cmake ..
make -j`nproc`
```

### run

```
cd ethercat_elmo/build/
sudo su
./test/elmo_test
```