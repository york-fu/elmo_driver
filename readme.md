# elmo driver

### describe

Elmo driver based on [soem v1.4.0](https://github.com/OpenEtherCATsociety/SOEM/tree/v1.4.0)

### download

```
# ssh
git clone ssh://git@www.lejuhub.com:10026/fuyou/elmo_driver.git
# https
git clone https://www.lejuhub.com/fuyou/elmo_driver.git
```

### run example

```
cd elmo_driver/
mkdir build && cd build
cmake -DEXAMPLE ..
make -j`nproc`
sudo su
./test/elmo_test
```