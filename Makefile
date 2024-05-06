init:
	git submodule update --init
	cd rocket-chip && git submodule update --init hardfloat cde

compile:
	mill -j 8 -i CoupledL2.compile

test-top-l2:
	mill -j 8 -i CoupledL2.test.runMain coupledL2.TestTop_L2 -td build

test-top-l2standalone:
	mill -j 8 -i CoupledL2.test.runMain coupledL2.TestTop_L2_Standalone -td build

test-top-l2l3:
	mill -j 8 -i CoupledL2.test.runMain coupledL2.TestTop_L2L3 -td build

test-top-l2l3l2:
	mill -j 8 -i CoupledL2.test.runMain coupledL2.TestTop_L2L3L2 -td build

test-top-fullsys:
	mill -j 8 -i CoupledL2.test.runMain coupledL2.TestTop_fullSys -td build

clean:
	rm -rf ./build

bsp:
	mill -j 8 -i mill.bsp.BSP/install

idea:
	mill -j 8 -i mill.scalalib.GenIdea/idea

reformat:
	mill -j 8 -i __.reformat

checkformat:
	mill -j 8 -i __.checkFormat
