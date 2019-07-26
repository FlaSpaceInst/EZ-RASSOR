#!/usr/bin/env bash
# Move the exported .dae meshes into the proper location within the repository.
# Written by Ronald Marrero.
MESH_DIR="../packages/simulation/ezrassor_sim_description/meshes"

mv *.dae "$MESHES_DIR"
