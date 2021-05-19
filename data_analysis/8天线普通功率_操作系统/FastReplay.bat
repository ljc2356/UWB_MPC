@echo off
for %%i in (.\data\20210508_11201\*.json) do (

	java -jar uwb-host.jar -c config/config.properties manager -d After%%i -f json  -o replay -C config/replay.properties -i %%i -f json -o tmp.txt
% java -jar uwb-host.jar -c config/config.properties manager -d After%%i -f json --single -o replay -C config/replay.properties -i %%i -f json -o tmp.txt %
	)
