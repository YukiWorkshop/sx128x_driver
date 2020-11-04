/*
    This file is part of SX128x Portable driver.
    Copyright (C) 2020 ReimuNotMoe

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "SX128x.hpp"

#include <cstdio>
#include <cinttypes>
#include <cassert>

int main() {
	SX128x::ModulationParams_t ModulationParams2;
	ModulationParams2.PacketType = SX128x::PACKET_TYPE_LORA;
	ModulationParams2.Params.LoRa.CodingRate = SX128x::LORA_CR_4_5;
	ModulationParams2.Params.LoRa.Bandwidth = SX128x::LORA_BW_1600;
	ModulationParams2.Params.LoRa.SpreadingFactor = SX128x::LORA_SF10;

	SX128x::PacketParams_t PacketParams2;
	PacketParams2.PacketType = SX128x::PACKET_TYPE_LORA;
	auto &l = PacketParams2.Params.LoRa;
	l.PayloadLength = 253;
	l.HeaderType = SX128x::LORA_PACKET_FIXED_LENGTH;
	l.PreambleLength = 12;
	l.Crc = SX128x::LORA_CRC_ON;
	l.InvertIQ = SX128x::LORA_IQ_NORMAL;

	auto ToA = SX128x::GetTimeOnAir(ModulationParams2, PacketParams2);
	printf("ToA: %" PRIu32 "\n", ToA);

	assert(ToA == 176);

	return 0;
}