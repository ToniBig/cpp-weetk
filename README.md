# cpp-weetk
A wee single header library to create vtk files

## Dependencies
Boost Any, Boost Variant

## Example Usage

### Unstructured Grid

``` cpp
    Points points = { { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 }, { 2, 1, 0 }, { 0, 0, 1 }, { 1, 0, 1 }, { 2, 0, 1 }, { 0, 1, 1 }, { 1, 1, 1 }, { 2, 1, 1 }, { 0, 1, 2 },
        { 1, 1, 2 }, { 2, 1, 2 }, { 0, 1, 3 }, { 1, 1, 3 }, { 2, 1, 3 }, { 0, 1, 4 }, { 1, 1, 4 }, { 2, 1, 4 }, { 0, 1, 5 }, { 1, 1, 5 }, { 2, 1, 5 }, { 0, 1, 6 }, { 1, 1, 6 }, { 2, 1, 6 } };

    Cells cells;
    cells.emplace_back( Hexahedron( { 0, 1, 4, 3, 6, 7, 10, 9 } ) );
    cells.emplace_back( Hexahedron( { 1, 2, 5, 4, 7, 8, 11, 10 } ) );
    cells.emplace_back( Tetra( { 6, 10, 9, 12 } ) );
    cells.emplace_back( Tetra( { 5, 11, 10, 14 } ) );
    cells.emplace_back( Polygon( { 15, 16, 17, 14, 13, 12 } ) );
    cells.emplace_back( Triangle_strip( { 18, 15, 19, 16, 20, 17 } ) );
    cells.emplace_back( Quad( { 22, 23, 20, 19 } ) );
    cells.emplace_back( Triangle( { 21, 22, 18 } ) );
    cells.emplace_back( Triangle( { 22, 19, 18 } ) );
    cells.emplace_back( Line( { 26, 25 } ) );
    cells.emplace_back( Vertex( { 24 } ) );

    std::vector<double> vector = { 1, 0, 0, 1, 1, 0, 0, 2, 0, 1, 0, 0, 1, 1, 0, 0, 2, 0, 1, 0, 0, 1, 1, 0, 0, 2, 0, 1, 0, 0, 1, 1, 0, 0, 2, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
        0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1 };

    std::vector<double> tensor = {
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8,
        0, 1, 2, 3, 4, 5, 6, 7, 8 };

    std::vector<size_t> scalar = range( 27 );

    Arrays pointData = { scalars( "Integers", std::move( scalar ) ), vectors( "Vectors", std::move( vector ) ) };

    UnstructuredGrid grid( std::move( points ), std::move( cells ) );

    weetk::VtkData data( "Example", std::move( grid ), std::move(pointData) );

    write( data, "unstructured" );
```

### Structured Grid

```cpp
StructuredGrid::Resolution resolution { 20, 30, 40 };
    Points points;
    std::vector<int> values;

    for ( const auto & z : range( resolution[2] + 1 ) )
    {
      for ( const auto & y : range( resolution[1] + 1 ) )
      {
        for ( const auto & x : range( resolution[0] + 1 ) )
        {
          points.emplace_back( Point { 1.0 * x, 1.0 * y, 1.0 * z } );
          values.push_back( x*x+y );
        }
      }
    }

    StructuredGrid grid( std::move( points ), resolution );

    weetk::VtkData data( "Structured Grid", std::move( grid ), { scalars( "values", std::move( values ) ) } );

    write( data, "structured" );

```
### Rectilinear Grid

```cpp
RectilinearGrid::Resolution resolution { 3, 3, 4 };
    RectilinearGrid::Coordinates coordinates;
    std::vector<int> values;

    for ( const auto & x : range( resolution[0]+1 ) )
    {
          coordinates[0].emplace_back( x*x );
    }
    for ( const auto & y : range( resolution[1]+1 ) )
    {
          coordinates[1].emplace_back( y*y*y );
    }
    for ( const auto & z : range( resolution[2]+1 ) )
    {
          coordinates[2].emplace_back( z );
    }

    for ( const auto & z : range( resolution[2] + 1 ) )
    {
      for ( const auto & y : range( resolution[1] + 1 ) )
      {
        for ( const auto & x : range( resolution[0] + 1 ) )
        {
          values.push_back( x*x+y-z );
        }
      }
    }

    RectilinearGrid grid( coordinates, resolution );

    weetk::VtkData data( "Rectilinear Grid", std::move( grid ), { scalars( "values", std::move( values ) ) } );

    write( data, "rectilinear" );
```
