#!/usr/bin/env bash
# Move the exported .dae meshes into the proper location within the repository.
# Written by Ronald Marrero.
MESH_DIR="../packages/simulation/ezrassor_sim_description/meshes"

mv EZRASSOR_BASE_UNIT.dae "$MESHES_DIR"/base_unit.dae
mv EZRASSOR_DRUM_ARM.dae "$MESHES_DIR"/drum_arm.dae
mv EZRASSOR_DRUM.dae "$MESHES_DIR"/drum.dae
mv EZRASSOR_WHEEL.dae "$MESHES_DIR"/wheel.dae
