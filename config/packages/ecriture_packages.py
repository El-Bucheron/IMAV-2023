"""
Description :
Programme servant à écrire les packages installés et leur version respective pour vérifier la présence des packages nécessaires pour faire tourner les codes.

Déroulement:
Le programme lit tous les packages installés et encode le nom et la version du package de la manière suivante : "nom version".
La chaîne de caractère obtenue est ensuite ajoutée sur une nouvelle ligne dans le fichier "packages_installes.txt"
"""

#Import du working_set contenant les noms et versions des packages installés  
from pkg_resources import working_set
#On convertit l'objet "working_set" en un tableau dont les éléments sont de la forme "nom_package version_package"
installed_packages_list = ["%s %s" % (i.key, i.version) for i in working_set]
#Ouverture du fichier dans lequel on écrit les informations
file = open("packages_installes.txt", "w")
#On lie les string du tableau "installed_packages_list" avec des sauts de ligne et on écrit cette chaîne dans le fichier "file"
file.write('\n'.join(sorted_installed_packages_list))
#Fermeture du fichier
file.close()