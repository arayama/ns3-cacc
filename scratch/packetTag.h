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
class packetTag : public Tag
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
  void Set (uint8_t value);
  /**
   * Get the tag value
   * \return the tag value.
   */
  uint8_t Get (void) const;
private:
  uint8_t m_simpleValue;  //!< tag value
};

TypeId 
packetTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::packetTag")
    .SetParent<Tag> ()
    .AddConstructor<packetTag> ()
    .AddAttribute ("SimpleValue",
                   "A simple value",
                   EmptyAttributeValue (),
                   MakeUintegerAccessor (&packetTag::Get),
                   MakeUintegerChecker<uint8_t> ())
  ;
  return tid;
}
TypeId 
packetTag::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}
uint32_t 
packetTag::GetSerializedSize (void) const
{
  return 1;
}
void 
packetTag::Serialize (TagBuffer i) const
{
  i.WriteU8 (m_simpleValue);
}
void 
packetTag::Deserialize (TagBuffer i)
{
  m_simpleValue = i.ReadU8 ();
}
void 
packetTag::Print (std::ostream &os) const
{
  os << "v=" << (uint32_t)m_simpleValue;
}
void 
packetTag::Set (uint8_t value)
{
  m_simpleValue = value;
}
uint8_t
packetTag::Get (void) const
{
  return m_simpleValue;
}


class test1:public packetTag
{

};
