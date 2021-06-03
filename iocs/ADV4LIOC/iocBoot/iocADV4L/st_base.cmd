# Must have loaded envPaths via st.cmd*

errlogInit(20000)

dbLoadDatabase("$(TOP)/dbd/ADV4LApp.dbd")
#simDetectorApp_registerRecordDeviceDriver(pdbbase)
ADV4LApp_registerRecordDeviceDriver(pdbbase)

# Prefix for all records
epicsEnvSet("PREFIX", "CAM1:")
# The port name for the detector
epicsEnvSet("PORT",   "CAM1")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximum image width; used to set the maximum size for this driver and for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "1024")
# The maximum image height; used to set the maximum size for this driver and for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "1024")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The maximum number of threads for plugins which can run in multiple threads
epicsEnvSet("MAX_THREADS", "8")

# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

#asynSetMinTimerPeriod(0.001) #Command not found

# The EPICS environment variable EPICS_CA_MAX_ARRAY_BYTES needs to be set to a value at least as large
# as the largest image that the standard arrays plugin will send.
# That value is $(XSIZE) * $(YSIZE) * sizeof(FTVL data type) for the FTVL used when loading the NDStdArrays.template file.
# The variable can be set in the environment before running the IOC or it can be set here.
# It is often convenient to set it in the environment outside the IOC to the largest array any client 
# or server will need.  For example 10000000 (ten million) bytes may be enough.
# If it is set here then remember to also set it outside the IOC for any CA clients that need to access the waveform record.  
# Do not set EPICS_CA_MAX_ARRAY_BYTES to a value much larger than that required, because EPICS Channel Access actually
# allocates arrays of this size every time it needs a buffer larger than 16K.
# Uncomment the following line to set it in the IOC.
#epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "10000000")
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", 90000000)

#Create a ADV4L driver
ADV4LConfig("$(PORT)", "/dev/video0")
#Load it's database
dbLoadRecords("$(ADV4L)/db/ADV4L.template","P=$(PREFIX),R=device0:,PORT=$(PORT),TIMEOUT=1,ADDR=0")

# Create a standard arrays plugin
NDStdArraysConfigure("image0", 5, 0, "$(PORT)", 0, 0)
# Allow for cameras up to 2048x2048x3 for RGB;
# For more examples see ADSimDetector/iocs/simDetectorIOC/iocBoot/iocSimDetector/st_base.cmd
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image0:,PORT=image0,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=12582912")

# Load all other plugins using commonPlugins.cmd
#< $(ADCORE)/iocBoot/commonPlugins.cmd
#set_requestfile_path("$(ADV4L)/ADV4LApp/Db")

#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,ASYN_TRACE_ERROR+ASYN_TRACE_WARNING+ASYN_TRACE_FLOW)
#asynSetTraceIOMask("FileNetCDF",0,2)
#asynSetTraceMask("FileNetCDF",0,255)
#asynSetTraceMask("FileNexus",0,255)
#asynSetTraceMask("SIM2",0,255)

iocInit()

# save things every thirty seconds
#create_monitor_set("auto_settings.req", 30, "P=$(PREFIX)")

# Set some defaults
dbpf CAM1:device0:Acquire 0
