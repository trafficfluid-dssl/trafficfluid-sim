set (PROJECT_LINK_LIBS_LF libtestLaneFree.so)
#include_directories( ../../../lane_free_control_library/build )
include_directories( ../../../lane_free_control_library/src)
set(microsim_cf_lf
   MSCFModel_LaneFree.cpp
   MSCFModel_LaneFree.h
)
add_library(microsim_cfmodelslf STATIC ${microsim_cf_lf})
target_link_libraries(microsim_cfmodelslf ${PROJECT_LINK_LIBS_LF})
set(microsim_cfmodels_STAT_SRCS
   MSCFModel.cpp
   MSCFModel.h
   MSCFModel_Daniel1.cpp
   MSCFModel_Daniel1.h
   MSCFModel_IDM.cpp
   MSCFModel_IDM.h
   MSCFModel_Kerner.cpp
   MSCFModel_Kerner.h
   MSCFModel_Krauss.cpp
   MSCFModel_Krauss.h
   MSCFModel_KraussOrig1.cpp
   MSCFModel_KraussOrig1.h
   MSCFModel_KraussPS.cpp
   MSCFModel_KraussPS.h
   MSCFModel_KraussX.cpp
   MSCFModel_KraussX.h
   MSCFModel_LaneFree.cpp
   MSCFModel_LaneFree.h
   MSCFModel_PWag2009.cpp
   MSCFModel_PWag2009.h
   MSCFModel_SmartSK.cpp
   MSCFModel_SmartSK.h
   MSCFModel_Wiedemann.cpp
   MSCFModel_Wiedemann.h
   MSCFModel_W99.cpp
   MSCFModel_W99.h
   MSCFModel_ACC.cpp
   MSCFModel_ACC.h
   MSCFModel_CACC.cpp
   MSCFModel_CACC.h
   MSCFModel_Rail.cpp
   MSCFModel_Rail.h
   CC_Const.h
   CC_VehicleVariables.cpp
   CC_VehicleVariables.h
   MSCFModel_CC.cpp
   MSCFModel_CC.h
   ParBuffer.h
)

add_library(microsim_cfmodels STATIC ${microsim_cfmodels_STAT_SRCS})
target_link_libraries(microsim_cfmodels PUBLIC microsim_cfmodelslf)
set_property(TARGET microsim_cfmodels PROPERTY PROJECT_LABEL "z_microsim_cfmodels")
