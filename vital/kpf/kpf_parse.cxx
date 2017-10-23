#include "kpf_parse.h"

#include <vital/util/tokenize.h>
#include <vector>
#include <stdexcept>

#include <vital/kpf/kpf_parse_utils.h>

#include <vital/logger/logger.h>

using std::istream;
using std::istringstream;
using std::string;
using std::vector;
using std::pair;
using std::make_pair;

kwiver::vital::kpf::private_endl_t kwiver::vital::kpf::record_text_writer::endl;

namespace { // anon
using namespace kwiver::vital::kpf;

static kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger( __FILE__ ) );

bool
packet_header_parser( const string& s, packet_header_t& packet_header, bool expect_colon )
{
  //
  // try to parse the header into a flag / tag / domain
  //

  header_parse_t h = parse_header( s, expect_colon );
  if (! std::get<0>(h) )
  {
    return false;
  }

  string tag_str( std::get<1>(h) );
  packet_style style = str2style( tag_str );
  if ( style == packet_style::INVALID )
  {
    LOG_ERROR( main_logger, "Bad packet style '" << tag_str << "'" );
    return false;
  }

  int domain( std::get<2>(h) );
  packet_header = packet_header_t( style, domain );
  return true;
}

bool
packet_parser( const vector<string>& tokens,
               packet_buffer_t& packet_buffer )
{
  size_t index(0), n( tokens.size() );

  while ( index < n )
  {
    packet_t p;
    if (! packet_header_parser( tokens[ index++ ],
                                p.header,
                                true ))
    {
      return false;
    }

    pair< bool, size_t > next = packet_payload_parser( index, tokens, p );
    if (! next.first )
    {
      return false;
    }
    index = next.second;

    packet_buffer.insert( make_pair( p.header, p ));
  }
  return true;
}


} // ...anon

namespace kwiver {
namespace vital {
namespace kpf {

text_reader_t
::text_reader_t()
  : is_set( false )
{
}

text_reader_t
::text_reader_t( const string& s )
  : is_set( false )
{
  this->init( s );
}

text_reader_t
::text_reader_t( const packet_header_t& h )
  : is_set( false ), header( h )
{
}

void
text_reader_t
::init( const string& s )
{
  if (! packet_header_parser( s, this->header, false ))
  {
    LOG_ERROR( main_logger, "Couldn't create a reader for packets of type '" << s << "'");
    return;
  }
}

void text_reader_t
::init( const packet_header_t& h )
{
  this->header = h;
}

void
text_reader_t
::set_from_buffer( const packet_t& p )
{
  this->is_set = true;
  this->packet = p;
}

packet_header_t
text_reader_t
::my_header() const
{
  return this->header;
}

text_reader_t&
text_reader_t
::set_domain( int d )
{
  this->is_set = false;
  this->header.domain = d;
  return *this;
}

pair< bool, packet_t >
text_reader_t
::get_packet()
{
  bool f = this->is_set;
  this->is_set = false;
  return make_pair( f, this->packet );
}

text_parser_t
::text_parser_t( istream& is )
  : packet_buffer( packet_header_cmp ), input_stream(is)
{
  this->reader_status = static_cast<bool>( is );
}


text_parser_t
::operator bool() const
{
  return this->reader_status;
}

vector< string >
text_parser_t
::get_meta_packets() const
{
  return this->meta_buffer;
}


bool
text_parser_t
::parse_next_line()
{
  //
  // loop over each line (throwing meta packets into the
  // meta-buffer) until either (a) non-meta packets have
  // been added or (b) getline() fails
  //

  bool non_meta_packets_added = false;
  packet_header_t meta_packet_h( packet_style::META );
  while ( ! non_meta_packets_added )
  {
    string s;
    if (! std::getline( this->input_stream, s ))
    {
      return false;
    }
    vector< string > tokens;
    ::kwiver::vital::tokenize( s, tokens, " ", true );

    //
    // pass the tokens off to the packet parser
    // store them in a local buffer to search for meta tags

    packet_buffer_t local_packet_buffer( packet_header_cmp );
    if (! packet_parser( tokens, local_packet_buffer ))
    {
      return false;
    }

    // pull any meta packets out
    packet_buffer_cit p;
    while ( (p = local_packet_buffer.find( meta_packet_h ))
           != local_packet_buffer.end() )
    {
      this->meta_buffer.push_back( p->second.meta.txt );
      local_packet_buffer.erase( p );
    }

    // if we have any packets left, push them into the
    // global packet buffer and set the flag
    if (! local_packet_buffer.empty())
    {
      this->packet_buffer.insert( local_packet_buffer.begin(), local_packet_buffer.end() );
      non_meta_packets_added = true;
    }
  }
  return non_meta_packets_added;
}

bool
text_parser_t
::verify_reader_status()
{
  if (! this->reader_status )
  {
    return false;
  }

  //
  // If the buffer is empty, read a 'record' (a line)
  // from the stream.
  //

  if (this->packet_buffer.empty())
  {
    if ( ! this->parse_next_line() )
    {
      this->reader_status = false;
      return false;
    }
  }
  return true;
}

pair< bool, packet_t >
text_parser_t
::transfer_kv_packet_from_buffer( const string& key )
{
  if (! this->verify_reader_status() )
  {
    return make_pair( false, packet_t() );
  }

  //
  // Look for a packet in the buffer which is (a) a kv packet,
  // and (b) its key value matches the parameter
  //

  auto probe =
    std::find_if( this->packet_buffer.cbegin(),
                  this->packet_buffer.cend(),
                  [key]( const std::pair< packet_header_t, packet_t>& p) -> bool {
                    return ((p.first.style == packet_style::KV) &&
                            (p.second.kv.key == key )); });

  if (probe == this->packet_buffer.end())
  {
    return make_pair( false, packet_t() );
  }

  //
  // remove the packet from the buffer and set the reader; we're done
  //

  auto ret = make_pair( true, probe->second );
  this->packet_buffer.erase( probe );
  return ret;
}


pair< bool, packet_t >
text_parser_t
::transfer_packet_from_buffer( const packet_header_t& h )
{
  if (! this->verify_reader_status() )
  {
    return make_pair( false, packet_t() );
  }

  //
  // if the head is invalid (i.e. the null reader) we're done
  //

  if (h.style == packet_style::INVALID)
  {
    return make_pair( true, packet_t() );
  }

  //
  // does the packet buffer contain what this reader is looking for?
  // if not, return false
  //

  auto probe = this->packet_buffer.find( h );
  if (probe == this->packet_buffer.end())
  {
    this->reader_status = false;
    return make_pair( false, packet_t() );
  }

  //
  // remove the packet from the buffer and set the reader; we're done
  //

  auto ret = make_pair( true, probe->second );
  this->packet_buffer.erase( probe );
  return ret;
}

bool
text_parser_t
::process_reader( text_reader_t& b )
{
  auto probe = this->transfer_packet_from_buffer( b.my_header() );
  if (probe.first)
  {
    b.set_from_buffer( probe.second );
  }
  return probe.first;
}

bool
text_parser_t
::process( text_reader_t& b )
{
  if ( this->reader_status )
  {
    bool okay = this->process_reader( b );
    this->reader_status = okay && this->reader_status;
  }
  return this->reader_status;
}

text_parser_t&
operator>>( text_parser_t& t,
            text_reader_t& b )
{
  t.process( b );
  return t;
}

bool
text_parser_t
::process( kpf_io_adapter_base& io )
{
  return this->process( io.text_reader );
}

text_parser_t& operator>>( text_parser_t& t,
                           kpf_io_adapter_base& io )
{
  return t >> io.text_reader;
}

text_parser_t&
operator>>( text_parser_t& t,
            const reader< canonical::bbox_t >& r )
{
  t.process( r.box_adapter.set_domain( r.domain ) );
  return t;
}

text_parser_t&
operator>>( text_parser_t& t,
            const reader< canonical::poly_t >& r )
{
  t.process( r.poly_adapter.set_domain( r.domain ) );
  return t;
}

text_parser_t&
operator>>( text_parser_t& t,
            const reader< canonical::id_t >& r )
{
  auto probe = t.transfer_packet_from_buffer( packet_header_t( packet_style::ID, r.domain ));
  if (probe.first)
  {
    r.id_ref = probe.second.id.d;
  }
  return t;
}

text_parser_t&
operator>>( text_parser_t& t,
            const reader< canonical::timestamp_t >& r )
{
  auto probe = t.transfer_packet_from_buffer( packet_header_t( packet_style::TS, r.domain ));
  if (probe.first)
  {
    switch (r.which)
    {
      case reader< canonical::timestamp_t >::to_int:
        r.int_ts = static_cast<int>( probe.second.timestamp.d );
        break;
      case reader< canonical::timestamp_t >::to_unsigned:
        r.unsigned_ts = static_cast<unsigned>( probe.second.timestamp.d );
        break;
      case reader< canonical::timestamp_t >::to_double:
        r.double_ts = probe.second.timestamp.d;
      break;
    }
  }
  return t;
}

text_parser_t&
operator>>( text_parser_t& t,
            const reader< canonical::kv_t >& r )
{
  auto probe = t.transfer_kv_packet_from_buffer( r.key );
  if (probe.first)
  {
    r.val = probe.second.kv.val;
  }
  return t;
}

text_parser_t&
operator>>( text_parser_t& t,
            const reader< canonical::conf_t >& r )
{
  auto probe = t.transfer_packet_from_buffer( packet_header_t( packet_style::CONF, r.domain ));
  if (probe.first)
  {
    r.conf = probe.second.conf.d;
  }
  return t;
}

text_parser_t&
operator>>( text_parser_t& t,
            const reader< canonical::meta_t >& r )
{
  auto probe = t.transfer_packet_from_buffer( packet_header_t( packet_style::META ));
  if (probe.first)
  {
    r.txt = probe.second.meta.txt;
  }
  return t;
}

record_text_writer&
operator<<( record_text_writer& w, const private_endl_t& )
{
  w.s << std::endl;
  return w;
}

record_text_writer&
operator<<( record_text_writer& w, const writer< canonical::id_t >& io)
{
  w.s << "id" << io.domain << ": " << io.id.d << " ";
  return w;
}

record_text_writer&
operator<<( record_text_writer& w, const writer< canonical::bbox_t >& io)
{
  w.s << "g" << io.domain << ": " << io.box.x1 << " " << io.box.y1 << " " << io.box.x2 << " " << io.box.y2 << " ";
  return w;
}

record_text_writer&
operator<<( record_text_writer& w, const writer< canonical::timestamp_t >& io)
{
  w.s << "ts" << io.domain << ": " << io.ts.d << " ";
  return w;
}

record_text_writer&
operator<<( record_text_writer& w, const writer< canonical::kv_t >& io)
{
  w.s << "kv: " << io.kv.key << " " << io.kv.val << " ";
  return w;
}

record_text_writer&
operator<<( record_text_writer& w, const writer< canonical::conf_t >& io)
{
  w.s << "conf" << io.domain << ": " << io.conf.d << " ";
  return w;
}

record_text_writer&
operator<<( record_text_writer& w, const writer< canonical::poly_t >& io)
{
  w.s << "poly" << io.domain << ": " << io.poly.xy.size() << " ";
  for (const auto& p : io.poly.xy )
  {
    w.s << p.first << " " << p.second << " ";
  }
  return w;
}

record_text_writer&
operator<<( record_text_writer& w, const writer< canonical::meta_t >& io)
{
  w.s << "meta: " << io.meta.txt;
  return w;
}


} // ...kpf
} // ...vital
} // ...kwiver
