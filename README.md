# Robot guitariste

Projet pluridisciplinaire d'équipe (ENIB, 4ᵉ année) : un robot qui se place sur les frettes d'une guitare et attaque la corde en rythme.

**Ma part** : modélisation des mécanismes et des chaînes cinématiques, outils de simulation en Python (trajectoires, vitesses, accélérations), analyse et optimisation des configurations, intégration mécanique / électronique / logiciel, documentation technique.

## Ce que contient le dossier

```
robot-guitariste/
├── firmware/
│   ├── control_guitarra/          micrologiciel STM32F411 (STM32CubeIDE), version de référence
│   └── versions-precedentes/      control_guitarra1, control_guitarra2
├── simulation/
│   ├── pythonsimulation/          simulation cinématique du bras (courbes, animation)
│   └── essais-python/             scripts d'essai successifs
├── cao-solidworks/                pièces, assemblages et animations SolidWorks
├── documents/                     rapport signé, comptes rendus, planification, vidéos d'analyse (Kinovea)
└── archives/                      export d'origine du projet
```

## Reprendre le projet

| Partie | Outil | Démarrer |
|---|---|---|
| Micrologiciel | STM32CubeIDE, carte Nucleo STM32F411 | *File → Import → Existing Projects* → `firmware/control_guitarra`, puis *Build* et *Debug* |
| Interface de pilotage | Python 3 + `pyserial` | `python firmware/control_guitarra/hmi_guitarra.py` (port série de la carte) |
| Simulation | Python 3 + `numpy`, `matplotlib` | `python simulation/pythonsimulation/robot_animation_full.py` |
| Mécanique | SolidWorks | ouvrir `cao-solidworks/Assemblage1.SLDASM` |

Les vidéos (`documents/*.mp4`) restent sur la machine : elles dépassent la taille acceptée par GitHub.

## État

Terminé dans le cadre du projet d'école. Présenté dans le portfolio (fiche « Robot guitariste »).

## À améliorer

- [ ] Nettoyer `firmware/` : ne garder qu'une version, documenter les commandes série (codes `X1…X12`, états `EN`, `N`, `G`…)
- [ ] `requirements.txt` pour la simulation et l'interface
- [ ] Une vidéo courte du robot qui joue, hébergée hors du dépôt
- [ ] Schéma du câblage (moteur pas-à-pas, solénoïde, alimentation)
