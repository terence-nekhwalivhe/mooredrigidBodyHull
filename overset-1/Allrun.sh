#!/bin/sh

#SBATCH --account=eleceng
#SBATCH --partition=ada
#SBATCH --nodes=1 --ntasks=40
#SBATCH --time=00:20:00
#SBATCH --job-name="mooredrigidBodyHull"
#SBATCH --mail-user=nkhadi001@myuct.ac.za
#SBATCH --mail-type=BEGIN,END,FAIL

# OpenFOAM
SINGULARITY_PATH=/opt/exp_soft/singularity-containers
FOAM_BASHRC=/opt/OpenFOAM/OpenFOAM-v2106/etc/bashrc

singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif blockMesh -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1
#singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif topoSet -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1
singularity run /opt/exp_soft/singularity-containers/openfoam/OpenFoam-2306-FoamMoor.sif snappyHexMesh -overwrite -case ~/OpenFOAM/nkhadi001-2306/foamMooring/tutorial/mooredrigidBodyHull/overset-1
