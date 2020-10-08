function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["rtDW"] = {file: "C:\\Users\\karst\\Desktop\\SSY191 Cybernetics\\GIT_SSY191\\quadrotor-lab-spring-2019-group_15_2019\\crazyflie-firmware\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	size: 16};
	 this.metricsArray.var["rtU"] = {file: "C:\\Users\\karst\\Desktop\\SSY191 Cybernetics\\GIT_SSY191\\quadrotor-lab-spring-2019-group_15_2019\\crazyflie-firmware\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	size: 80};
	 this.metricsArray.var["rtY"] = {file: "C:\\Users\\karst\\Desktop\\SSY191 Cybernetics\\GIT_SSY191\\quadrotor-lab-spring-2019-group_15_2019\\crazyflie-firmware\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	size: 56};
	 this.metricsArray.fcn["atan2"] = {file: "C:\\Program Files\\MATLAB\\R2018b\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["crazyflie_initialize"] = {file: "C:\\Users\\karst\\Desktop\\SSY191 Cybernetics\\GIT_SSY191\\quadrotor-lab-spring-2019-group_15_2019\\crazyflie-firmware\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["crazyflie_step"] = {file: "C:\\Users\\karst\\Desktop\\SSY191 Cybernetics\\GIT_SSY191\\quadrotor-lab-spring-2019-group_15_2019\\crazyflie-firmware\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	stack: 176,
	stackTotal: 176};
	 this.metricsArray.fcn["crazyflie_terminate"] = {file: "C:\\Users\\karst\\Desktop\\SSY191 Cybernetics\\GIT_SSY191\\quadrotor-lab-spring-2019-group_15_2019\\crazyflie-firmware\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["sqrt"] = {file: "C:\\Program Files\\MATLAB\\R2018b\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data; }; 
	 this.codeMetricsSummary = '<a href="crazyflie_metrics.html">Global Memory: 152(bytes) Maximum Stack: 176(bytes)</a>';
	}
CodeMetrics.instance = new CodeMetrics();
