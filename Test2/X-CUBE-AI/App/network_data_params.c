/**
  ******************************************************************************
  * @file    network_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Mon Feb 20 11:24:52 2023
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "network_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_network_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_network_weights_array_u64[41] = {
  0x3f0dd80f3e7a99bfU, 0x3f0fd5a73f6a1330U, 0x3f5db5003f2a4291U, 0x3c97c7e2bedd229bU,
  0x3e28eb1fbe3fc0c2U, 0xbe3ca78bbdcbe634U, 0xbee6924f3d25a161U, 0x3d4c667bbf429de8U,
  0x3d90833ebe76f842U, 0x3c04bdc0be8f55e5U, 0xbeb06b773e87e054U, 0xbdd9b5d8bf1652ecU,
  0xbe6d38c03e4b1a54U, 0xbea74cb83f070637U, 0xbe0312603e0aa0dcU, 0x3f1b85c33f914153U,
  0x3e90a98d3f7e14a6U, 0x3f7f0fa73efb8e08U, 0x3f07a7373f4ec014U, 0x3f20e31f3d3d1e7eU,
  0x3f87252d3df5769dU, 0x3f1609de3f7b1cb0U, 0x3f55b8203f44e9fbU, 0x3f1afc753ec6a915U,
  0x3e9e6d3abef72516U, 0xbf17bd883b63c9d7U, 0xbf291e833de409ccU, 0x3e9bb6d23e63c7eeU,
  0x3f50af5b3d95d0daU, 0x3f5ae1293f63d98fU, 0xbe79997f3f465ae1U, 0xbe292a4aU,
  0x3f4bfe0100000000U, 0x3f402ab13f08dbc9U, 0x3f076453bd67f6ccU, 0xbeae1c1c3f521d11U,
  0xbd66dbd0bd668210U, 0x3f52a146bf37b68eU, 0x3f5a6e723f301538U, 0x3f4be44d3cf436ebU,
  0x3eb99bc2U,
};


ai_handle g_network_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_network_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

