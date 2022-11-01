g++ -Wall -g -std=c++11 ../sw_src/event_timer.cpp ../sw_src/xilinx_ocl_helper.cpp ../sw_src/twobody.cpp -o twobody -I${XILINX_XRT}/include/ -L${XILINX_XRT}/lib/-rdynamic /opt/xilinx/xrt/lib/libxrt_core.so /opt/xilinx/xrt/lib/libxilinxopencl.so -lOpenCL -pthread -lrt -lstdc++
emconfigutil --platform xilinx_u200_gen3x16_xdma_2_202110_1 --nd 1
v++ -c -t sw_emu --platform xilinx_u200_gen3x16_xdma_2_202110_1 --config ../u200.cfg -k twobody -I../hw_src ../hw_src/twobody.cpp -o twobody.xo
v++ -l -t sw_emu --platform xilinx_u200_gen3x16_xdma_2_202110_1 --config ../u200.cfg ./twobody.xo -o twobody.xclbin
