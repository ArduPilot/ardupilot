macro(MacroSetDefault VAR DEFAULT)
	if (NOT DEFINED ${VAR})
		set(${VAR} ${DEFAULT})
	endif()
endmacro(MacroSetDefault)

