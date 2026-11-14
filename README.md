# Commande vectorielle (FOC) d'un PMSM sous Simulink

Modélisation d'un moteur synchrone à aimants permanents dans le repère dq, réglage analytique des régulateurs de courant et validation en simulation. Chaque résultat est comparé à une valeur calculée à la main.

Ce dépôt couvre la première étape du projet : la **boucle de courant avec découplage**. La boucle de vitesse, le défluxage et le portage sur STM32 suivront.

## Contenu

```
.
├── params_pmsm.m               Paramètres du moteur et gains des régulateurs
├── models/
│   ├── etapeA_RL.slx           Circuit RL de l'axe q, rotor bloqué
│   └── foc_boucle_courant.slx  Moteur dq complet + régulateurs + découplage
├── figures/                    Courbes de validation
└── docs/
    ├── rapport_foc_pmsm.tex    Rapport détaillé (source LaTeX)
    └── rapport_foc_pmsm.pdf    Rapport compilé
```

## Démarche

Le modèle est construit en quatre étapes, avec un essai de validation à chacune.

| Étape | Ajout | Vérification |
|---|---|---|
| A | Circuit RL de l'axe q, rotor bloqué | Constante de temps L/R = 2 ms |
| B | Régulateur PI de courant | Boucle fermée du premier ordre, τ = 1/ωc |
| C | Mécanique, f.é.m. et feedforward | Erreur de traînage sans feedforward, limite de tension |
| D | Axe d, termes croisés et découplage | id maintenu à zéro pendant l'accélération |

## Modèle et réglage

Équations du moteur en convention moteur :

```
vd = Rs·id + Ld·did/dt − ωe·Lq·iq
vq = Rs·iq + Lq·diq/dt + ωe·(Ld·id + ψf)
Te = 1.5·p·[ψf·iq + (Ld − Lq)·id·iq]
J·dωm/dt = Te − TL − B·ωm ,  ωe = p·ωm
```

Loi de commande, un PI par axe avec découplage :

```
vd* = PI(id* − id) − ωe·Lq·iq
vq* = PI(iq* − iq) + ωe·(Ld·id + ψf)
```

Réglage par compensation pôle-zéro : `Kp = L·ωc`, `Ki = Rs·ωc`. La boucle fermée devient un premier ordre de bande passante ωc. Avec fc = 1 kHz pour un découpage à 20 kHz, le retard de 1,5·Ts coûte environ 27° de marge de phase (63° restants).

La tension est limitée à Vdc/√3 (zone linéaire SVPWM), avec anti-windup sur les PI et saturation en aval du découplage.

## Résultats

| Essai | Calculé | Simulé |
|---|---|---|
| B : constante de temps de la boucle de courant | 0,159 ms | 0,160 ms |
| C : iq sans feedforward, pendant l'accélération | 1,85 A | 1,85 à 1,9 A |
| C : iq final à vide | 0,06 A | environ 0,06 A |
| D : id sans découplage, pendant l'accélération | +0,03 A | environ +0,03 A |
| D : id avec découplage | 0 A | 0 A |

### Étape B : réponse de la boucle de courant

![Réponse de iq à un échelon de consigne](figures/iq_etapeB.png)

### Étape C : effet du feedforward de f.é.m.

| Sans feedforward | Avec feedforward |
|---|---|
| ![iq sans feedforward](figures/iq_sans_ff.png) | ![iq avec feedforward](figures/iq_avec_ff.png) |

Sans feedforward, le PI court après une f.é.m. qui croît en rampe (480 V/s) et garde une erreur égale à pente / Ki. Dans les deux cas, le courant s'effondre lorsque la f.é.m. rejoint la tension maximale : c'est la limite que le défluxage repoussera.

### Étape D : effet du découplage

| Avec découplage | Sans découplage |
|---|---|
| ![id avec découplage](figures/id_avec_decouplage.png) | ![id sans découplage](figures/id_sans_decouplage.png) |

Le PI seul finit par rejeter le couplage entre axes, mais avec une erreur pendant les transitoires de vitesse. Le découplage la supprime sans attendre l'action intégrale.

## Lancer la simulation

1. Ouvrir MATLAB dans le dossier du dépôt.
2. Exécuter `params_pmsm.m`.
3. Ouvrir `models/foc_boucle_courant.slx` et lancer la simulation (durée 0,1 s).
4. Les deux interrupteurs manuels activent ou coupent le feedforward de l'axe q et le découplage de l'axe d.

Réglages du solveur : pas variable `ode45`, pas maximal `1e-5` s, tolérance relative `1e-4`.

Version utilisée : MATLAB/Simulink R20XX (à préciser).

## Limites actuelles

- Régulateur continu, sans retard de calcul ni de PWM.
- Onduleur idéal, saturation par axe et non sur le module de la tension.
- Mesures parfaites, paramètres du moteur supposés connus.
- Moteur à pôles lisses (Ld = Lq), sans couple de réluctance.

## Suite prévue

- [ ] Boucle de vitesse (PI extérieur, sortie iq*)
- [ ] Défluxage et limitation du module de la tension
- [ ] Discrétisation du régulateur à 20 kHz
- [ ] Modèle abc avec transformées de Clarke/Park, onduleur et SVPWM
- [ ] Étude de robustesse aux erreurs de paramètres
- [ ] Génération de code (Embedded Coder) et essais sur banc STM32

## Auteur

Bangaly Conte
