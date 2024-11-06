Se han realizado cambios en el código base de algunos ficheros de SOFA por problemas generados al no estar actualizado.

- visualmodel.py -> Eliminación de la función list, ya que generaba conflictos con la lectura de los arrays de numpy (np.float64)
- elasticmaterialobject.py -> Eliminación de la función list, ya que generaba conflictos con la lectura de los arrays de numpy (np.float64)