idf_component_register(
		    	SRCS "main.c"
			"../libs/myI2C/my_i2c.c"
			"../libs/myHTTP/my_http.c"
			"../libs/cJSON/cJSON.c"
			"../libs/myJSON/my_JSON.c"
			"../libs/MAX44009/MAX44009.c"
			"../libs/ML8511/ML8511.c"
			"../libs/mySNTP/my_SNTP.c"

                    	INCLUDE_DIRS
			"../libs" 
			"../inc"
			"../libs/myI2C"	
			"../libs/myHTTP"
			"../libs/cJSON"	    
			"../libs/myJSON"
			"../libs/MAX44009"
			"../libs/ML8511"
			"../libs/mySNTP"
			)