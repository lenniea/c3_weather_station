set DEFINES=-DWIND_SENSOR -DTEMP_SENSOR -DHUMID_SENSOR -DPRESS_SENSOR
rem
rem change #theme to cpp comment
rem
sed "s|#theme|//#theme|g" %1 > %1.tmp
rem
rem process file using CPP preprocessor
rem
cpp -C -P %DEFINES% -o %1.strip %1.tmp
rem
rem change cpp comment back to #theme
rem
sed "s|//#theme|#theme|g" %1.strip > data/web/%1
rem delete temporary files
rm %1.tmp
rm %1.strip
