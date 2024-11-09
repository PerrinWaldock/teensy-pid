
"""
unfinished file to auto generate the updateparams code
"""



filename = "updateparams"



functionname = "int8_t updateparams(ParamStruct pms, char* string)"

paramdetails = [{"name": "kp", "keyword": None, "convfn": "atof({})", "printfn": None, "units": None, "cc": None, "desc": None},
	{"name": "ki", "keyword": None, "convfn": "atof({})", "printfn": None, "units": None, "cc": None, "desc": None},
	{"name": "kd", "keyword": None, "convfn": "atof({})", "printfn": None, "units": None, "cc": None, "desc": None}, 
	{"name": "setpoint", "keyword":"sp", "convfn": "atoi({})", "printfn": None, "units": None, "cc": None, "desc": None}, 
	{"name": "setpoint", "keyword":"sv", "convfn": "volts2int(atof({}), ADC_BITS)", "printfn": "int2volts({}, ADC_BITS)", "units": "V", "cc": None, "desc": None}, 
	{"name": "setpoint", "keyword":"sv", "convfn": "volts2int(atof({}), ADC_BITS)", "printfn": "int2volts({}, ADC_BITS)", "units": "V", "cc": None, "desc": None}, 
	{"name": "minoutput", "keyword":"lo", "convfn": "volts2int(atof({}), PWM_BITS)", "printfn": "int2volts({}, PWM_BITS)", "units": "V", "cc": "LIMITED_SETPOINT", "desc": "Output for Low Setpoint"}, 
	{"name": "maxoutput", "keyword":"lo", "convfn": "volts2int(atof({}), PWM_BITS)", "printfn": "int2volts({}, PWM_BITS)", "units": "V", "cc": "LIMITED_SETPOINT", "desc": "Output for High Setpoint"}, 
	]


with open(filename+".h", "w") as f:
	f.write("#include \"arduinopid.h\"\n\n")
	f.write("struct* ParamStruct {\n")
	for p in paramdetails:
		f.write("{type:} {name:};\n")
	f.write("};\n\n")
	f.write(functionname+";\n")


with open(filename+".c", "w") as f:
	tab = 0
	def w(s):
		tab -= s.count('}')
		f.write("\t"*tab + s +"\n")
		tab += s.count('{')

	w("#include \"{}\"\n".format(filename+".h"))
	w(functionname)
	w("{")
	w("char* peql = strchr(string, '=');")
	w("if (peql)")
	w("{")
	w("*peql = '\0';")
	w("char* datastr = peql+1;")

	p = paramdetails[0]
	if p["cc"] != None:
		w("#if{}".format(p["cc"]))
	if p["keyword"] != None:
		w("if(!strcmp(string, \"{keyword:}\"))".format(p))
	else:
		w("if(!strcmp(string, \"{name:}\"))".format(p))
	w("{")
	w("*pms->{} = {}".format(p["name"], p["convfn"].format("datastr")))
	if p["desc"] != None:
		w("Serial.print(\"{desc:}=\")".format(p))
	else:
		w("Serial.print(\"{name:}=\")".format(p))
	if p["printfn"] != None:
		w("Serial.print({})".format(p["printfn"].format((p["name"]))))
	else:
		w("Serial.print({:name})".format(p))
	if p["units"] != None:
		w("Serial.println(\"({units:})\")".format(p))
	else:
		w("Serial.println()")
	w("}")
	if p["cc"] != None:
		w("#endif")

	for p in paramdetails[1:]:
		if p["cc"] != None:
			w("#if{}".format(p["cc"]))
		if p["keyword"] != None:
			w("else if(!strcmp(string, \"{keyword:}\"))".format(p))
		else:
			w("else if(!strcmp(string, \"{name:}\"))".format(p))
		w("{")
		w("*pms->{} = {}".format(p["name"], p["convfn"].format("datastr")))
		if p["desc"] != None:
			w("Serial.print(\"{desc:}=\")".format(p))
		else:
			w("Serial.print(\"{name:}=\")".format(p))
		if p["printfn"] != None:
			w("Serial.print({})".format(p["printfn"].format((p["name"]))))
		else:
			w("Serial.print({:name})".format(p))
		if p["units"] != None:
			w("Serial.println(\"({units:})\")".format(p))
		else:
			w("Serial.println()")
		w("}")
		if p["cc"] != None:
			w("#endif")

	w("}")
	w("}")
