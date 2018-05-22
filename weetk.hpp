#pragma once

#include <boost/any.hpp>
#include <boost/variant.hpp>

#include <array>
#include <fstream>
#include <sstream>
#include <vector>

namespace weetk {

struct Point {
  double x, y, z;
};

enum class CellType {
  VTK_VERTEX = 1,
  VTK_POLY_VERTEX = 2,
  VTK_LINE = 3,
  VTK_POLY_LINE = 4,
  VTK_TRIANGLE = 5,
  VTK_TRIANGLE_STRIP = 6,
  VTK_POLYGON = 7,
  VTK_PIXEL = 8,
  VTK_QUAD = 9,
  VTK_TETRA = 10,
  VTK_VOXEL = 11,
  VTK_HEXAHEDRON = 12,
  VTK_WEDGE = 13,
  VTK_PYRAMID = 14,
};

template<size_t N>
using IndexArray = std::array<size_t,N>;
using IndexVector = std::vector<size_t>;

template<typename PointIdContainer, CellType Type>
struct GenericCell {
  GenericCell( PointIdContainer&& input ) :
          pointIds( std::move( input ) ){
  }

  GenericCell( PointIdContainer const & input ) :
          pointIds( input ){
  }

  PointIdContainer pointIds;
  const CellType type = Type;
};

using Vertex =GenericCell<IndexArray<1>,CellType::VTK_VERTEX>;
using Poly_vertex =GenericCell<IndexVector,CellType::VTK_POLY_VERTEX >;
using Line =GenericCell<IndexArray<2>,CellType::VTK_LINE >;
using Poly_line =GenericCell<IndexVector,CellType::VTK_POLY_LINE >;
using Triangle =GenericCell<IndexArray<3>,CellType::VTK_TRIANGLE >;
using Triangle_strip =GenericCell<IndexVector,CellType::VTK_TRIANGLE_STRIP >;
using Polygon =GenericCell<IndexVector,CellType::VTK_POLYGON >;
using Pixel =GenericCell<IndexArray<4>,CellType::VTK_PIXEL >;
using Quad =GenericCell<IndexArray<4>,CellType::VTK_QUAD >;
using Tetra =GenericCell<IndexArray<4>,CellType::VTK_TETRA >;
using Voxel =GenericCell<IndexArray<8>,CellType::VTK_VOXEL >;
using Hexahedron =GenericCell<IndexArray<8>,CellType::VTK_HEXAHEDRON >;
using Wedge =GenericCell<IndexArray<6>,CellType::VTK_WEDGE >;
using Pyramid =GenericCell<IndexArray<5>,CellType::VTK_PYRAMID >;

using Cell = boost::variant<
Vertex,
Poly_vertex ,
Line ,
Poly_line ,
Triangle ,
Triangle_strip,
Polygon ,
Pixel ,
Quad ,
Tetra ,
Voxel ,
Hexahedron ,
Wedge ,
Pyramid >;

namespace detail {

template<typename T>
constexpr const char* type( );

template<> constexpr const char* type<int8_t>( ){
  return "Int8";
}
template<> constexpr const char* type<int16_t>( ){
  return "Int16";
}
template<> constexpr const char* type<int32_t>( ){
  return "Int32";
}
template<> constexpr const char* type<int64_t>( ){
  return "Int64";
}

template<> constexpr const char* type<uint8_t>( ){
  return "UInt8";
}
template<> constexpr const char* type<uint16_t>( ){
  return "UInt16";
}
template<> constexpr const char* type<uint32_t>( ){
  return "UInt32";
}
template<> constexpr const char* type<uint64_t>( ){
  return "UInt64";
}

template<> constexpr const char* type<float_t>( ){
  return "Float32";
}
template<> constexpr const char* type<double_t>( ){
  return "Float64";
}

} // namespace detail

template<typename T>
struct Array {
  using data_type=std::vector<T>;

  Array( std::string name,
         ushort number_of_components,
         data_type && data ) :
          name_( std::move( name ) ),
          number_of_components_( number_of_components ),
          data_( std::forward<data_type>( data ) ){
    if ( ( this->data_.size( ) % number_of_components ) != 0 ) throw std::runtime_error( "Data size must be multiple of number of components" );
  }

  const std::string & name( ) const{
    return name_;
  }

  ushort number_of_components( ) const{
    return number_of_components_;
  }

  const data_type & data( ) const{
    return data_;
  }

private:
  std::string name_;
  ushort number_of_components_;
  data_type data_;
};

template<typename T>
const char* type( Array<T> const & array ){
  return detail::type<T>( );
}

template<typename T>
void write( std::ostream& stream,
            Array<T> const & array,
            std::string prefix = "" ){
  for ( size_t i = 0; i < array.data( ).size( ); ++i ) {
    stream << ( i % array.number_of_components( ) ? "" : prefix );
    stream << array.data( )[i];
    stream << ( ( i + 1 ) % array.number_of_components( ) ? ' ' : '\n' );
  } // end of i-loop
}

using ArrayVariant = boost::variant<
Array<int8_t>,
Array<int16_t>,
Array<int32_t>,
Array<int64_t>,
Array<uint8_t>,
Array<uint16_t>,
Array<uint32_t>,
Array<uint64_t>,
Array<float_t>,
Array<double_t>>;

using Arrays=std::vector<ArrayVariant>;

struct ArrayWriter : boost::static_visitor<> {
  ArrayWriter( std::ostream& stream,
               std::string prefix = "" ) :
          stream_( stream ),
          prefix_( prefix ){
  }

  template<typename T>
  void operator()( Array<T> const & array ){
    write( stream_, array, prefix_ );
  }

private:
  std::ostream& stream_;
  std::string prefix_;
};

inline void write( std::ostream& stream,
                   ArrayVariant const & array,
                   std::string prefix = "" ){
  ArrayWriter writer { stream, prefix };
  boost::apply_visitor( writer, array );
}

template<typename T>
Array<T> scalars( std::string name,
                  std::vector<T> && data ){
  return Array<T> { std::move( name ), 1, std::forward<std::vector<T>>( data ) };
}

template<typename T>
Array<T> vectors( std::string name,
                  std::vector<T> && data ){
  return Array<T> { std::move( name ), std::forward<std::vector<T>>( data ) };
}

template<typename T>
Array<T> tensors( std::string name,
                  std::vector<T> && data ){
  return Array<T> { std::move( name ), 9, std::forward<std::vector<T>>( data ) };
}

struct DataAttributeVisitor : boost::static_visitor<std::string> {
  template<typename T>
  std::string operator()( Array<T> const & data ) const{
    std::string attribute;

    switch ( data.number_of_components( ) ) {
    case 1:
      attribute = "Scalars";
      break;
    case 3:
      attribute = "Vectors";
      break;
    case 9:
      attribute = "Tensors";
      break;
    default:
      return
      {};
      break;
    }
    return attribute + "=\"" + data.name( ) + "\"";
  }
};

struct ArrayAttributeVisitor : boost::static_visitor<std::string> {
  template<typename T>
  std::string operator()( Array<T> const & data ) const{
    std::stringstream stream;
    stream << " NumberOfComponents=\"" << data.number_of_components( ) << "\"";
    stream << " type=\"" << type( data ) << "\"";
    stream << " Name=\"" << data.name( ) << "\"";
    return stream.str( );
  }
};

inline std::string dataAttribute( ArrayVariant const & array ){
  static const DataAttributeVisitor visitor;

  std::string result { " " };
  result += boost::apply_visitor( visitor, array );
  return result;
}

inline std::string arrayAttribute( ArrayVariant const & array ){
  static const ArrayAttributeVisitor visitor;

  std::string result;
  result += boost::apply_visitor( visitor, array );
  return result;
}

inline std::string dataAttributes( Arrays const & arrays ){
  std::string result;
  for ( const auto & array : arrays ) {
    result += dataAttribute( array );
  }
  return result;
}

using Points = std::vector<Point>;
using Cells = std::vector<Cell>;

struct UnstructuredGrid {
  UnstructuredGrid( Points && points,
                    Cells && cells ) :
          points_( std::move( points ) ),
          cells_( std::move( cells ) ){
  }

  UnstructuredGrid( Points const & points,
                    Cells const & cells ) :
          points_( points ),
          cells_( cells ){
  }

  const Points & points( ) const{
    return points_;
  }

  const Cells & cells( ) const{
    return cells_;
  }

private:
  Points points_;
  Cells cells_;
};

struct StructuredGrid {
  using Resolution = std::array<size_t,3>;

  StructuredGrid( Points && points,
                  Resolution resolution ) :
          points_( std::forward<Points>( points ) ),
          resolution_( std::move( resolution ) ){
    if ( points_.size( ) != ( ( resolution[0] + 1 ) * ( resolution[1] + 1 ) * ( resolution[2] + 1 ) ) ) throw std::runtime_error( "Number of points does not match resolution" );
  }

  const Points & points( ) const{
    return points_;
  }

  const Resolution & resolution( ) const{
    return resolution_;
  }

private:
  Points points_;
  Resolution resolution_;
};

struct RectilinearGrid {
  using Coordinates=std::array<std::vector<double>,3>;
  using Resolution = std::array<size_t,3>;

  RectilinearGrid( Coordinates coordinates,
                   Resolution resolution ) :
          coordinates_( std::move( coordinates ) ),
          resolution_( std::move( resolution ) ){
//    if ( coordinates_[0].size( )+coordinates_[1].size( )+coordinates_[2].size( ) != ( ( resolution[0] + 1 ) + ( resolution[1] + 1 ) + ( resolution[2] + 1 ) ) ) throw std::runtime_error( "Number of points does not match resolution" );
  }

  const Coordinates & coordinates( ) const{
    return coordinates_;
  }

  const Resolution & resolution( ) const{
    return resolution_;
  }

private:
  Coordinates coordinates_;
  Resolution resolution_;
};

using Grid = boost::variant<UnstructuredGrid,StructuredGrid,RectilinearGrid>;

namespace detail {

using PointIds = std::vector<size_t>;

struct PointIdQuery : boost::static_visitor<PointIds> {
  template<typename PointIdContainer, CellType Type>
  PointIds operator()( GenericCell<PointIdContainer, Type> const & cell ) const{
    PointIds pointIds { std::begin( cell.pointIds ), std::end( cell.pointIds ) };
    return pointIds;
  }

  static PointIds evaluate( Cell const & cell ){
    return boost::apply_visitor( PointIdQuery { }, cell );
  }
};

struct CellTypeQuery : boost::static_visitor<size_t> {
  template<typename PointIdContainer, CellType Type>
  size_t operator()( GenericCell<PointIdContainer, Type> const & cell ) const{
    return static_cast<size_t>( cell.type );
  }

  static size_t evaluate( Cell const & cell ){
    return boost::apply_visitor( CellTypeQuery { }, cell );
  }
};

inline void dataToXml( std::ostream& stream,
                       const Arrays& data,
                       const std::string& attribute ){
  stream << "<" << attribute << dataAttributes( data ) << ">" << "\n";
  for ( const auto& array : data ) {
    stream << "<DataArray" << arrayAttribute( array ) << ">" << "\n";
    write( stream, array );
    stream << "</DataArray>" << "\n";
  }
  stream << "</" << attribute << ">" << "\n";
}

inline void cellsToXml( std::ostream& stream,
                        const Cells& cells ){
  stream << "<Cells>" << "\n";

  stream << "<DataArray type=\"UInt64\" Name=\"connectivity\" format=\"ascii\"" << ">" << "\n";
  std::vector<size_t> idCounts;
  for ( const auto& cell : cells ) {
    const auto& pointIds = detail::PointIdQuery::evaluate( cell );
    std::copy( std::begin( pointIds ), std::end( pointIds ), std::ostream_iterator<size_t>( stream, " " ) );
    idCounts.push_back( pointIds.size( ) );
    stream << "\n";
  }
  stream << "</DataArray>" << "\n";

  stream << "<DataArray type=\"UInt64\" Name=\"offsets\" format=\"ascii\"" << ">" << "\n";
  size_t offset = 0;
  for ( const auto& idCount : idCounts ) {
    offset += idCount;
    stream << offset;
    stream << "\n";
  }
  stream << "</DataArray>" << "\n";

  stream << "<DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\"" << ">" << "\n";
  for ( const auto& cell : cells ) {
    stream << detail::CellTypeQuery::evaluate( cell );
    stream << "\n";
  }
  stream << "</DataArray>" << "\n";

  stream << "</Cells>" << "\n";
}

inline void pointsToXml( std::ostream& stream,
                         const Points & points ){
  // Points ---------------------------------------------------------------------
  stream << "<Points>" << "\n";
  stream << "<DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\"" << ">" << "\n";
  for ( const auto& point : points ) {
    stream << point.x << " " << point.y << " " << point.z << "\n";
  }
  stream << "</DataArray>" << "\n";
  stream << "</Points>" << "\n";
}

inline void coordinatesToXml( std::ostream& stream,
                              const RectilinearGrid::Coordinates & coordinates ){
  // Coordinates ----------------------------------------------------------------
  stream << "<Coordinates>" << "\n";
  stream << "<DataArray type=\"Float64\" NumberOfComponents=\"1\" format=\"ascii\"" << ">" << "\n";
  for ( const auto& coordinate : coordinates[0] ) {
    stream << coordinate << "\n";
  }
  stream << "</DataArray>" << "\n";
  stream << "<DataArray type=\"Float64\" NumberOfComponents=\"1\" format=\"ascii\"" << ">" << "\n";
  for ( const auto& coordinate : coordinates[1] ) {
    stream << coordinate << "\n";
  }
  stream << "</DataArray>" << "\n";
  stream << "<DataArray type=\"Float64\" NumberOfComponents=\"1\" format=\"ascii\"" << ">" << "\n";
  for ( const auto& coordinate : coordinates[2] ) {
    stream << coordinate << "\n";
  }
  stream << "</DataArray>" << "\n";
  stream << "</Coordinates>" << "\n";
}

inline void write( std::ostream & stream,
                   UnstructuredGrid const & grid,
                   Arrays const & pointData,
                   Arrays const & cellData ){
  std::string gridType = "UnstructuredGrid";

  stream << "<?xml version=\"1.0\"?>" << "\n";

  stream << "<VTKFile type=\"" << gridType << "\" version=\"1.0\" byte_order=\"LittleEndian\">" << "\n";
  stream << "<" << gridType << ">" << "\n";

  stream << "<Piece NumberOfPoints=\"" << grid.points( ).size( ) << "\" NumberOfCells=\"" << grid.cells( ).size( ) << "\">\n";

  pointsToXml( stream, grid.points( ) );
  cellsToXml( stream, grid.cells( ) );
  dataToXml( stream, pointData, "PointData" );
  dataToXml( stream, cellData, "CellData" );

  // ----------------------------------------------------------------------------

  stream << "</Piece>" << "\n";
  stream << "</" << gridType << ">" << "\n";
  stream << "</VTKFile>" << "\n";
}

inline void write( std::ostream & stream,
                   StructuredGrid const & grid,
                   Arrays const & pointData,
                   Arrays const & cellData ){
  stream << "<?xml version=\"1.0\"?>" << "\n";

  std::string gridType = "StructuredGrid";
  auto resolution = grid.resolution( );
  std::stringstream extent;
  extent << "0 " << resolution[0] << " 0 " << resolution[1] << " 0 " << resolution[2];

  stream << "<VTKFile type=\"" << gridType << "\" version=\"1.0\" byte_order=\"LittleEndian\">" << "\n";
  stream << "<" << gridType << " WholeExtent=\"" << extent.str( ) << "\">" << "\n";

  stream << "<Piece Extent=\"" << extent.str( ) << "\">" << "\n";

  pointsToXml( stream, grid.points( ) );
  dataToXml( stream, pointData, "PointData" );
  dataToXml( stream, cellData, "CellData" );

  // ----------------------------------------------------------------------------

  stream << "</Piece>" << "\n";
  stream << "</" << gridType << ">" << "\n";
  stream << "</VTKFile>" << "\n";
}

inline void write( std::ostream & stream,
                   RectilinearGrid const & grid,
                   Arrays const & pointData,
                   Arrays const & cellData ){
  stream << "<?xml version=\"1.0\"?>" << "\n";

  std::string gridType = "RectilinearGrid";
  auto resolution = grid.resolution( );
  std::stringstream extent;
  extent << "0 " << resolution[0] << " 0 " << resolution[1] << " 0 " << resolution[2];

  stream << "<VTKFile type=\"" << gridType << "\" version=\"1.0\" byte_order=\"LittleEndian\">" << "\n";
  stream << "<" << gridType << " WholeExtent=\"" << extent.str( ) << "\">" << "\n";

  stream << "<Piece Extent=\"" << extent.str( ) << "\">" << "\n";

  dataToXml( stream, pointData, "PointData" );
  dataToXml( stream, cellData, "CellData" );
  coordinatesToXml( stream, grid.coordinates( ) );

  // ----------------------------------------------------------------------------

  stream << "</Piece>" << "\n";
  stream << "</" << gridType << ">" << "\n";
  stream << "</VTKFile>" << "\n";
}

struct GridWriter : boost::static_visitor<> {
  GridWriter( std::ostream& stream,
              Arrays const & pointData,
              Arrays const & cellData ) :
          stream_( stream ),
          pointData_( pointData ),
          cellData_( cellData ){
  }

  template<typename T>
  void operator()( T const & grid ) const{
    write( stream_, grid, pointData_, cellData_ );
  }
private:
  std::ostream& stream_;
  Arrays pointData_;
  Arrays cellData_;
};

inline void writeGrid( std::ostream& stream,
                       Grid const & grid,
                       Arrays const & pointData,
                       Arrays const & cellData ){
  boost::apply_visitor( GridWriter { stream, pointData, cellData }, grid );
}

struct ExtensionQuery : boost::static_visitor<std::string> {
  std::string operator()( UnstructuredGrid const & grid ) const{
    return "vtu";
  }

  std::string operator()( StructuredGrid const & grid ) const{
    return "vts";
  }

  std::string operator()( RectilinearGrid const & grid ) const{
    return "vtr";
  }

  static std::string evaluate( Grid const & grid ){
    return boost::apply_visitor( ExtensionQuery { }, grid );
  }
};

} // namespace detail

struct VtkData {
  VtkData( std::string name,
           Grid && grid,
           Arrays && pointData = Arrays { },
           Arrays && cellData = Arrays { } ) :
          name_( std::move( name ) ),
          grid_( std::move( grid ) ),
          pointData_( std::move( pointData ) ),
          cellData_( std::move( cellData ) ){
  }

  Grid const & grid( ) const{
    return grid_;
  }

  Arrays const & pointData( ) const{
    return pointData_;
  }

  Arrays const & cellData( ) const{
    return cellData_;
  }

  std::string const & name( ) const{
    return name_;
  }

  void write( std::string const & fileName ) const{
    std::string fileNameWithExtension = fileName + "." + detail::ExtensionQuery::evaluate( grid_ );
    std::ofstream file( fileNameWithExtension );

    detail::writeGrid( file, grid_, pointData_, cellData_ );
  }

private:
  std::string name_;
  Grid grid_;
  Arrays pointData_;
  Arrays cellData_;
};

}
// namespace weetk
