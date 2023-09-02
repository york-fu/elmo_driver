# ECM of ELMO

### Describe

ECM of ELMO based on [SOEM](https://github.com/OpenEtherCATsociety/SOEM), commit: 9b6ebd6e43e03be461c5eab803a74786319a14db

### Run example

```
mkdir build
cd build
cmake -DEXAMPLE=ON ..
make -j`nproc`

# Use root run
./example/ecm_elmo_example
```