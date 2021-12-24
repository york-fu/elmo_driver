# elmo driver

### describe

Elmo driver based on [soem v1.4.0](https://github.com/OpenEtherCATsociety/SOEM/tree/v1.4.0)

### run example

```
mkdir build && cd build
cmake -DEXAMPLE ..
make -j`nproc`

sudo su
./test/elmo_test
```
