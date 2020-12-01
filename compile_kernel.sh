export ARCH=arm64
export CROSS_COMPILE=/media/tom/hdd-seco-4tb/arm/rockchip/px30/prebuilts/gcc/linux-x86/aarch64/gcc-linaro-6.3.1-2017.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-

rmdir OUT

cp arch/arm64/configs/lec-px30_config .config
make Image dtbs modules -j20

mkdir OUT
cp -v arch/arm64/boot/dts/rockchip/px30-evb-ddr3-v10-linux.dtb OUT
cp -v arch/arm64/boot/Image OUT

#dtb -> px30-evb-ddr3-v10-linux.dtb