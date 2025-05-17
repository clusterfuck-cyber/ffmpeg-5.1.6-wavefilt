# wavefilt - Filtre de Forme d'Onde Avancé pour FFmpeg

## Aperçu

Le filtre `wavefilt` est un filtre vidéo avancé pour FFmpeg qui applique des effets de déplacement d'onde configurables aux images vidéo. Contrairement au filtre `waves` de base, il offre un contrôle précis sur les motifs d'onde, les méthodes d'interpolation et les paramètres d'animation.

## Fonctionnalités

- Multiples modes d'onde : sinusoïdale, circulaire, radiale, carrée et personnalisée
- Contrôle précis de l'amplitude, de la fréquence et de la phase pour les dimensions X et Y
- Trois méthodes d'interpolation : plus proche voisin, bilinéaire et bicubique
- Mise en miroir des bords optionnelle
- Équations d'onde personnalisées via l'évaluation d'expressions
- Optimisations spécifiques au CPU (SIMD x86 pour AVX2)
- Optimisations spécifiques pour les processeurs Intel Skylake (i5-6600 et similaires)

## Prérequis

- FFmpeg (compatible avec les versions 6.0 et supérieures)
- Pour les optimisations SIMD x86 :
  - Processeur Intel avec support AVX2 (Haswell ou plus récent)
  - Pour les optimisations Skylake complètes : CPU Intel Skylake ou plus récent

## Installation

Le filtre est inclus dans FFmpeg. Pour compiler FFmpeg avec ce filtre spécifiquement activé et optimisé pour Skylake :

```sh
./configure --enable-gpl --enable-libx264 --enable-alsa --enable-filter=wavefilt --cpu=skylake --extra-cflags='-march=skylake -mtune=skylake -O3' && make -j4
```

Ou pour une compilation standard qui inclut le filtre :

```sh
./configure --enable-filter=wavefilt && make -j4
```

## Utilisation

### Syntaxe de Base

```sh
ffmpeg -i entree.mp4 -vf "wavefilt=<options>" sortie.mp4
```

### Options

- `mode` : Mode de motif d'onde [0-4] (par défaut : 0)
  - 0 : Onde sinusoïdale
  - 1 : Onde circulaire
  - 2 : Onde radiale
  - 3 : Onde carrée
  - 4 : Onde personnalisée (nécessite dx_expr et dy_expr)
  - *Min : 0, Max : 4*

- `amplitude_x` : Amplitude pour la dimension X 
  - *Par défaut : 10.0, Min : 0.0, Max : 1000.0*
  - Contrôle le déplacement maximal en pixels sur l'axe x

- `amplitude_y` : Amplitude pour la dimension Y 
  - *Par défaut : 10.0, Min : 0.0, Max : 1000.0*
  - Contrôle le déplacement maximal en pixels sur l'axe y

- `frequency_x` : Fréquence pour la dimension X 
  - *Par défaut : 0.1, Min : 0.0, Max : 10.0*
  - Des valeurs plus élevées créent plus de cycles d'onde sur la largeur

- `frequency_y` : Fréquence pour la dimension Y 
  - *Par défaut : 0.1, Min : 0.0, Max : 10.0*
  - Des valeurs plus élevées créent plus de cycles d'onde sur la hauteur

- `phase_x` : Phase pour la dimension X 
  - *Par défaut : 0.0, Min : -π (-3.14159), Max : π (3.14159)*
  - Contrôle la position de départ du motif d'onde sur l'axe x

- `phase_y` : Phase pour la dimension Y 
  - *Par défaut : 0.0, Min : -π (-3.14159), Max : π (3.14159)*
  - Contrôle la position de départ du motif d'onde sur l'axe y

- `speed` : Vitesse d'animation 
  - *Par défaut : 1.0, Min : -100.0, Max : 100.0*
  - Les valeurs négatives inversent la direction du mouvement

- `decay` : Facteur de décroissance de l'amplitude depuis le centre 
  - *Par défaut : 0.0, Min : 0.0, Max : 10.0*
  - Des valeurs plus élevées font diminuer les ondes plus rapidement avec la distance du centre

- `center_x` : Coordonnée X du centre pour les ondes circulaires/radiales 
  - *Par défaut : 0.5, Min : 0.0, Max : 1.0*
  - Coordonnée normalisée (0.0 = bord gauche, 1.0 = bord droit)

- `center_y` : Coordonnée Y du centre pour les ondes circulaires/radiales 
  - *Par défaut : 0.5, Min : 0.0, Max : 1.0*
  - Coordonnée normalisée (0.0 = bord supérieur, 1.0 = bord inférieur)

- `interp_mode` : Méthode d'interpolation [0-2] 
  - *Par défaut : 1 (Bilinéaire), Min : 0, Max : 2*
  - 0 : Plus proche voisin (le plus rapide, qualité la plus basse)
  - 1 : Bilinéaire (bon équilibre entre vitesse et qualité)
  - 2 : Bicubique (qualité la plus élevée, utilisation CPU la plus intensive)

- `mirror` : Activer la mise en miroir des bords 
  - *Par défaut : 1 (activé), Min : 0, Max : 1*
  - Quand activé, évite les bordures noires en mettant en miroir les pixels aux limites de l'image

- `dx_expr` : Expression de déplacement X personnalisée 
  - *Par défaut : null (pas d'expression)*
  - Chaîne d'expression AVExpr pour calculer le déplacement x quand mode=4

- `dy_expr` : Expression de déplacement Y personnalisée 
  - *Par défaut : null (pas d'expression)*
  - Chaîne d'expression AVExpr pour calculer le déplacement y quand mode=4

- `time_base` : Temps de base en secondes pour la vitesse d'animation 
  - *Par défaut : 1.0, Min : 0.001, Max : 100.0*
  - Contrôle l'échelle de temps pour l'animation

### Variables pour les Expressions Personnalisées

Les variables suivantes peuvent être utilisées dans `dx_expr` et `dy_expr` :

- `x`, `y` : Coordonnées du pixel courant
- `w`, `h` : Largeur et hauteur de l'entrée
- `t` : Horodatage actuel en secondes
- `cx`, `cy` : Coordonnées du centre
- `a` : Amplitude (valeur combinée)
- `d` : Distance depuis le centre (pour les modes circulaire/radial)

## Exemples

### Onde Sinusoïdale Simple

```sh
ffmpeg -i entree.mp4 -vf "wavefilt=amplitude_x=20:amplitude_y=10:frequency_x=0.05:frequency_y=0.05:speed=0.5" sortie.mp4
```

### Effet d'Ondulation Circulaire

```sh
ffmpeg -i entree.mp4 -vf "wavefilt=mode=1:amplitude_x=15:frequency_x=0.2:speed=0.8:decay=0.1" sortie.mp4
```

### Motif d'Onde Personnalisé

```sh
ffmpeg -i entree.mp4 -vf "wavefilt=mode=4:dx_expr='sin(x/10+t*5)*10':dy_expr='cos(y/20+t*3)*15':interp_mode=2" sortie.mp4
```

### Effet Eau de Haute Qualité

```sh
ffmpeg -i entree.mp4 -vf "wavefilt=mode=1:amplitude_x=8:amplitude_y=8:frequency_x=0.2:frequency_y=0.2:speed=0.3:interp_mode=2:mirror=1" sortie.mp4
```

## Notes de Performance

- Le filtre inclut du code optimisé AVX2 pour l'interpolation bilinéaire et bicubique qui améliore significativement les performances sur les CPU compatibles
- Optimisations spécifiques pour l'architecture Intel Skylake via l'option `--cpu=skylake` et les drapeaux de compilateur appropriés
- L'interpolation bicubique (interp_mode=2) offre la plus haute qualité mais est plus intensive en CPU
- Pour le traitement en temps réel, considérez l'utilisation de l'interpolation du plus proche voisin (interp_mode=0) ou bilinéaire (interp_mode=1)
- Pour des valeurs d'amplitude élevées (>100), vous pourriez avoir besoin d'ajuster le paramètre `mirror` pour éviter les artefacts aux bords de l'image

## Implémentation Technique

Le filtre effectue une cartographie de déplacement en calculant des motifs d'onde et en les appliquant aux coordonnées de l'image source. Composants clés :

1. Générateurs de motifs d'onde pour différents modes (sinusoïdal, circulaire, etc.)
2. Transformation des coordonnées de pixel basée sur les valeurs de déplacement
3. Diverses méthodes d'interpolation pour l'échantillonnage des pixels source
4. Implémentations optimisées AVX2 des fonctions d'interpolation
5. Système d'évaluation d'expressions pour les motifs d'onde personnalisés

## Licence

GPLv2+/LGPLv2.1+, comme le reste de FFmpeg.

## Auteur

Copyright (c) 2025 Développeurs FFmpeg
