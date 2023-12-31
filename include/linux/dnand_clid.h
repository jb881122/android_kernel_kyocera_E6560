#ifndef DNAND_CLID_H
#define DNAND_CLID_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/types.h>

typedef enum
{
  DNAND_ID_KERNEL_0 = 0,
  DNAND_ID_KERNEL_1,
  DNAND_ID_KERNEL_2,
  DNAND_ID_KERNEL_3,
  DNAND_ID_KERNEL_4,
  DNAND_ID_KERNEL_5,
  DNAND_ID_KERNEL_6,
  DNAND_ID_KERNEL_7,
  DNAND_ID_KERNEL_8,
  DNAND_ID_KERNEL_9,
  DNAND_ID_KERNEL_10,
  DNAND_ID_KERNEL_11,
  DNAND_ID_KERNEL_12,
  DNAND_ID_KERNEL_13,
  DNAND_ID_KERNEL_14,
  DNAND_ID_KERNEL_15,
  DNAND_ID_KERNEL_16,
  DNAND_ID_KERNEL_17,
  DNAND_ID_KERNEL_18,
  DNAND_ID_KERNEL_19,
  DNAND_ID_KERNEL_20,
  DNAND_ID_KERNEL_21,
  DNAND_ID_KERNEL_22,
  DNAND_ID_KERNEL_23,
  DNAND_ID_KERNEL_24,
  DNAND_ID_KERNEL_25,
  DNAND_ID_KERNEL_26,
  DNAND_ID_KERNEL_27,
  DNAND_ID_KERNEL_28,
  DNAND_ID_KERNEL_29,

  DNAND_ID_KERNEL_51 = 51,
  DNAND_ID_KERNEL_52,
  DNAND_ID_KERNEL_53,
  DNAND_ID_KERNEL_54,
  DNAND_ID_KERNEL_55,
  DNAND_ID_KERNEL_56,
  DNAND_ID_KERNEL_57,
  DNAND_ID_KERNEL_58,
  DNAND_ID_KERNEL_59,
  DNAND_ID_KERNEL_60,
  DNAND_ID_KERNEL_61,
  DNAND_ID_KERNEL_62,
  DNAND_ID_KERNEL_63,

  DNAND_ID_ENUM_MAX
} dnand_id_enum_type;

#endif
