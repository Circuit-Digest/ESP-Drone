idf_component_register(SRC_DIRS "BasicMathFunctions"
                        "CommonTables"
                        "ComplexMathFunctions"
                        "ControllerFunctions"
                        "FastMathFunctions"
                        "FilteringFunctions"
                        "MatrixFunctions"
                        "StatisticsFunctions"
                        "TransformFunctions"
                        INCLUDE_DIRS "include"
                    )

target_compile_options(${COMPONENT_LIB} PUBLIC "-fno-strict-aliasing")