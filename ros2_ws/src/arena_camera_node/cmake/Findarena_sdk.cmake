set(_LOG_LVL DEBUG) # Logging level
set(_LOG_LVL_FRMT "-- [ ${_LOG_LVL} ] ")

# Path to Arena SDK root directory for Windows
if(WIN32)
    set(_arena_sdk_conf "C:/Program Files/Lucid Vision Labs/Arena SDK")
else()
    set(_arena_sdk_conf "/etc/ld.so.conf.d/Arena_SDK.conf")
endif()

if(EXISTS ${_arena_sdk_conf})

    ###### --------------------------------------------------------------------
    # ROOT
    ######
    
    if(WIN32)
        # Set the root path directly for Windows
        set(arena_sdk_installation_root "${_arena_sdk_conf}")
    else()
        # For Linux, extract the directory as before
        execute_process(
            COMMAND bash -c "dirname $(head -n 1 \"/etc/ld.so.conf.d/Arena_SDK.conf\")"
            OUTPUT_VARIABLE arena_sdk_installation_root
            # ENCODING UTF8
        )
        string(STRIP ${arena_sdk_installation_root} arena_sdk_installation_root)
    endif()
    
    message(${_LOG_LVL_FRMT} "arena_sdk_installation_root = ${arena_sdk_installation_root}")

    ######### -----------------------------------------------------------------
    # INCLUDE
    #########
    
    set(arena_sdk_INCLUDE_DIRS
        "${arena_sdk_installation_root}/GenICam/library/CPP/include"
        "${arena_sdk_installation_root}/include/Arena"
        "${arena_sdk_installation_root}/include/Save")
    set(arena_sdk_INCLUDES ${arena_sdk_INCLUDE_DIRS})

    ###### --------------------------------------------------------------------
    # LIBS
    ######
    
    if(WIN32)
        # Windows paths for the Arena SDK libraries
        set(arena_sdk_LIBS
            "${arena_sdk_installation_root}/lib64/Arena/Arena_v140.lib"
            "${arena_sdk_installation_root}/lib64/Arena/Save_v140.lib"
            "${arena_sdk_installation_root}/lib64/Arena/GenTL_LUCID_v140.lib"
            
            ## GenICam Libraries
            "${arena_sdk_installation_root}/GenICam/library/CPP/lib/Win64_x64/GCBase_MD_VC140_v3_3_LUCID.lib"
            "${arena_sdk_installation_root}/GenICam/library/CPP/lib/Win64_x64/GenApi_MD_VC140_v3_3_LUCID.lib"
            
        )
    else()
        # Linux paths
        if(EXISTS "${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc421_v3_0.so")
            set(ArenaSDK_Build "Linux64_x64_pre_54")
        elseif(EXISTS "${arena_sdk_installation_root}/GenICam/library/lib/Linux64_x64/libGCBase_gcc54_v3_3_LUCID.so")
            set(ArenaSDK_Build "Linux64_x64_54")
        elseif(EXISTS "${arena_sdk_installation_root}/GenICam/library/lib/Linux64_ARM/libGCBase_gcc54_v3_3_LUCID.so")
            set(ArenaSDK_Build "Linux64_ARM")
        else()
            message(FATAL_ERROR "LUCID GenICam not found. Please reinstall ArenaSDK. Contact support@thinklucid.com.")
        endif()
        
        # Add Linux library paths as before (this code snippet omits for brevity)
    endif()

    set(arena_sdk_LIBRARIES ${arena_sdk_LIBS})
    set(arena_sdk_FOUND true)
    message(${_LOG_LVL_FRMT} "arena_sdk_LIBRARIES = ${arena_sdk_LIBRARIES}")
    message(${_LOG_LVL_FRMT} "arena_sdk_FOUND = ${arena_sdk_FOUND}")

else()
    message(FATAL_ERROR "ArenaSDK is not installed. Please install it using the script provided by LUCID support team.")
endif()
