/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006,2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include <iostream>

using namespace ns3;

/**
 * \ingroup network
 * A simple example of an Tag implementation
 */
class posXTag : public Tag
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  // these are our accessors to our tag structure
  /**
   * Set the tag value
   * \param value The tag value.
   */
  void Set (double value);
  /**
   * Get the tag value
   * \return the tag value.
   */
  double Get (void) const;
private:
  double m_simpleValue;  //!< tag value
};

TypeId 
posXTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::posXTag")
    .SetParent<Tag> ()
    .AddConstructor<posXTag> ()
    .AddAttribute ("SimpleValue",
                   "A simple value",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&posXTag::Get),
                   MakeUintegerChecker<uint8_t> ())
  ;
  return tid;
}
TypeId 
posXTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t 
posXTag::GetSerializedSize (void) const
{
  return 1;
}
void 
posXTag::Serialize (TagBuffer i) const
{
  i.WriteDouble (m_simpleValue);
}
void 
posXTag::Deserialize (TagBuffer i)
{
  m_simpleValue = i.ReadDouble ();
}
void 
posXTag::Print (std::ostream &os) const
{
  os << "v=" << m_simpleValue;
}
void 
posXTag::Set (double value)
{
  m_simpleValue = value;
}
double 
posXTag::Get (void) const
{
  return m_simpleValue;
}

