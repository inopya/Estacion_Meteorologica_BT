# Estacion_Meteorologica_BT

/*
//##########################################################
//# ****************************************************** #
//# *           DOMOTICA PARA PRINCIPIANTES              * #
//# *   EST. METEO. con BT y almacenamiento en EEPROM    * #
//# *         (acceso al SENSOR DHT11 sin libreria)      * #
//# *                                                    * #
//# *            Autor: Eulogio López Cayuela            * #
# *                                                    * #
# *          Versión 2.1     Fecha: 21/01/2017         * #
# ****************************************************** #
##########################################################
*/

/*   
    NOTAS SOBRE ESTA VERSION
    
    ===== OPCIONES DISPONIBLES ===== 
 - Muestra datos completos por BT 
 - Muestra datos simplificados en el LCD (presion relativa, temperatura y humedad)
 - Permite cambiar la altitud mediante comandos BT (para el calculo de presion relativa)
 - Guarda la informacion de altitud programada en las dos primeras direcciones de la EEPROM (0,1)
 - Guarda la Temp MAx y Min en las posiciones 2 y 3 de la EEPROM
 - Guarda registros de temperaturas, presion y humedad apartir de la sexta posicion (5) de la EEPROM
 - Comando BT (m) para guardar una marca horaria y los datos de ese momento en la EEPROM
 - listado de datos guardados atraves de BT
 - Listado de datos guardados >>> al puerto serie (se puede dar la orden  por BT o serie)
 - Borrado completo de la EEPROM (respetando las 5 primeras posiciones para no borrar la altitud)
 - Borrado parcial de la EEPROM, solo las posiciones que contienen datos de P,T y H. (respeta la altitud)
 - Posibilidad de cancelar el proceso de marca horaria, cambio de altitud o borrados de EEPROM mediante el comando 'x'
 - Posibilidad de envio de una muestra al puerto Serie. Comando serie (*)
 - Posibilidad de envio de los datos almacenados en la eeprom, al puerto serie. Comando serie (+)
 - ELIMINADA el calculo de la sensacion termica para liberar espacio y mejorar la estabilidad
 - ELIMINADOS muchos caracteres superfluos de los mensajes para liberar 
    casi 200 bytes de RAM y mas de 1500 de memoria de programa evitandose asi inestabilidades
 - AÑADIDA opcion 'v' para mostras solo los ultimos (12) registros   
 - AÑADIDAS banderas para indicar USO y DISPONIBILIDAD de EEPROM evitando que el
    puntero EEPROM se reinicie cuando alcanza el punto maximo y deciciendo si se desean 
    almacenar datos o no en la EEPROM, protegiendola de escrituras innecesarias.
 - AÑADIDA rutina para la prediccon de tiempo a unas cuantas (3) horas vista (basadas en tendencias de presion)
 - AÑADIDA bandera para el control y registro de temperaturas max y min
*/
