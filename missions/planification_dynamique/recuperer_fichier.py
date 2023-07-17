#Ce programme récupère un fichier .csv fourni par Mission Planner et trie les données pour récupérer le temps Us, la latitude, la longitude et l'altitude.

#Librairies nécessaires

import csv
import pandas as pd

# L'utilisateur entre le chemin du fichier qu'il souhaite trier et le nom du fichier trié
input_file = input("Entrez le nom du fichier d'entrée (ex: output.csv) : ")
output_file = input("Entrez le nom du fichier de sortie (ex: tableau_ok.csv) : ")

# Données de délection des colonnes et d'attribution des titres
selected_columns = [1, 10, 11, 12]  # Indices des colonnes à sélectionner (commençant à 0)
column_titles = ["Time US", "Latitude", "Longitude", "Altitude"]  # Titres des colonnes sélectionnées


df = pd.read_csv(input_file) # Lecture du fichier que l'utilisateur souhaite trier
selected_df = df.iloc[:, selected_columns] # Sélection des colonnes 
selected_df.columns = column_titles # Attribution des titres
selected_df.to_csv(output_file, index=False) # Création d'un fichier .csv
