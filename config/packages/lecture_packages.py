"""
Description :
Programme servant à vérifier, à partir d'une liste de référence, la présence et la version des packages.
Cela permet de nous assurer que tous les packages nécessaires à l'utilisation des codes sont présents

Déroulement:
La liste des packages de référence est stockée dans le fichier "packages_installes.txt".
Chaque ligne de ce fichier représente le nom du package et sa version encodés de la manière suivante : "nom_package version_package".
On sépare ces informations en deux tableaux, l'un contenant les noms de packages de référence et l'autre contenant la version du package.
On récupère ensuite les packages installés localement et on les compare à la liste de référence.
Pour chaque package de référence, on vérifie qu'il est présent dans la liste des packages installés et qu'ils sont de la même version.
"""

#Import du working_set contenant les noms et versions des packages installés  
from pkg_resources import working_set


#Ouverture du fichier de référence
file = open("packages_installes.txt", "r")
#Lecture des données
read_data = file.readlines()
#Fermeture du fichier
file.close()

#Création des array contenant respectivement les noms de packages de référence et leur version
reference_packages_name_array = []
reference_packages_version_array = []
#On sépare les noms et versions des packages de référence et les place dans le tableau adéquat
for line in read_data:
    line = line[0:-1].split() #On supprime le '\n' de fin de ligne avec le line[0:-1]
    reference_packages_name_array.append(line[0])
    reference_packages_version_array.append(line[1])

#Création des array contenant respectivement les noms de packages locaux et leur version
local_packages_name_array = []
local_packages_version_array = []
#On sépare les noms et versions des packages locaux et les place dans le tableau adéquat
for local_package in working_set:
    local_packages_name_array.append(local_package.key)
    local_packages_version_array.append(local_package.version)


#On traite tous
for reference_package in reference_packages_name_array:
    #Si le package de référence pointé par la boucle n'est pas présent dans le tableau des packages locaux, on affiche le nom du package manquant
    if reference_package not in local_packages_name_array:
        print("Package " + reference_package + " non présent")
    #Si le package est présent on vérifie sa version
    else:
        #On récupère la version du package de référence pointé et celle du package installé
        local_package_version = local_packages_version_array[local_packages_name_array.index(reference_package)]
        reference_package_version = reference_packages_version_array[reference_packages_name_array.index(reference_package)]
        #Si les versions sont différentes, on affiche la version local et la version attendue
        if(reference_package_version != local_package_version):
            print("Package " + reference_package + " en version " + local_package_version + ". Version attendue : " + reference_package_version) 