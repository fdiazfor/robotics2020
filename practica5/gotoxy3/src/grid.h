//
// Created by salabeta on 24/11/20.
//

#ifndef GOTOXY_GRID_H
#define GOTOXY_GRID_H

#include <QGraphicsItem>

template<typename HMIN, HMIN hmin, typename WIDTH, WIDTH width, typename TILE, TILE tile>
class Grid
{
    public:
        Grid()
        {
            array.resize((int)(width/tile));
            for (auto &row : array)
                row.resize((int)(width/tile));
            int k=0;
            for (int i = hmin; i < width/2; i += tile, k++)
            {
                int l=0;
                for (int j = hmin; j < width/2; j += tile, l++)
                {
                    array[k][l] = Value{false, nullptr, nullptr, i, j};
                }
            }
        };

        struct Value
        {
            bool occupied = false;
            QGraphicsRectItem * paint_cell = nullptr;
            QGraphicsTextItem * text_cell = nullptr;
            int cx, cy;
            //Añadir elemento gráfico para pintar texto
            //Crear en el constructor e inicializar a comillas comillas
            int dist = -1; //dist vecinos
        };

        std::vector<std::vector<Value>> array;
//Hacer método que me de lso vecinos de una celda que te de vecinos libres()que no están ocupadosos
//El metodo devuelve una lista con los vecinos que sean libres y que no estén en la lista 1.
//hacer un metodo con while que se llama desde donde se hace el click
        void create_graphic_items(QGraphicsScene &scene)
        {
            auto fondo = QColor("LightGreen"); fondo.setAlpha(40);
            QFont font("Bavaria");
            font.setPointSize(40);
            font.setWeight(QFont::TypeWriter);
            for (auto &row : array)
                for (auto &elem : row)
                {
                    elem.paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("DarkGreen")),
                                                    QBrush(fondo));
                    elem.paint_cell->setPos(elem.cx, elem.cy);
                    elem.text_cell = scene.addText("-1", font);
                    elem.text_cell->setPos(elem.cx-tile/2, elem.cy-tile/2);
                    QTransform transform(elem.text_cell->transform());
                    qreal m11 = transform.m11();    // Horizontal scaling
                    qreal m12 = transform.m12();    // Vertical shearing
                    qreal m13 = transform.m13();    // Horizontal Projection
                    qreal m21 = transform.m21();    // Horizontal shearing
                    qreal m22 = transform.m22();    // vertical scaling
                    qreal m23 = transform.m23();    // Vertical Projection
                    qreal m31 = transform.m31();    // Horizontal Position (DX)
                    qreal m32 = transform.m32();    // Vertical Position (DY)
                    qreal m33 = transform.m33();    // Addtional Projection Factor
                    // Vertical flip
                    m22 = -m22;
                    // Write back to the matrix
                    transform.setMatrix(m11, m12, m13, m21, m22, m23, m31, m32, m33);
                    // Set the items transformation
                    elem.text_cell->setTransform(transform);
                }
        }
    /*
 * Inicializamos el array a false, osea, no ocupadas.
 */
    void inicializate()
    {
        for (int i = 0; i < this->tam; ++i) {
            for (int j = 0; j < this->tam; ++j) {
                this->array[i][j] = false;
            }
        }
    }

public:
    /**
     * modificamos en funcion de v la coordenada x,z
     * @param x
     * @param z
     * @param v
     */
    void set_Value(int x, int y, Value v)
    {
       auto [nx, ny] = this->transformar(x,y);
       this->array[nx][ny] = v;
    }
    /**
     * devolvemos el valor de la coordenada x,z
     * @param x
     * @param z
     * @return
     */
    Value get_value(int x, int y)
    {
        auto [nx, ny] = transformar(x,y);
        return  this->array[nx][ny];
    }

    std::list<Value> get_Neighbors(Value v, int dist){
        std::list<Value> list;
        for(auto[dk, dl]: list_neighbors){
            int x = v.cx + dk;
            int y = v.cy + dl;
            bool limits = x > -width/2 && x < width/2  && y > -width/2 && y < width/2;
            if(limits && !this->array[x][y].occupied && this->array[x][y].dist == -1) {
                this->array[x][y].dist = dist;
                this->array[x][y].text_cell.setText(QString::number(dist));
                list.push_front(this->array[x][y]);
            }
        }
        return list;
    }

    int get_Dist(int x, int y){
        auto [nx, ny] = transformar(x, y);
        return this->array[nx][ny].dist;
    }

    bool get_Occupied(int x, int y){
        auto [nx, ny] = transformar(x, y);
        return this->array[nx][ny].occupied;
    }

    void set_Occupied(int x, int y){
        auto[nx, ny] = this->transformar(x, y);
        this->array[nx][ny].occupied = true;
        this->array[nx][ny].paint_cell->setBrush(QBrush(QColor("Red")));
    }

    void set_Dist(int x, int y, int ndist){
        auto[nx, ny] = this->transformar(x, y);
        this->array[nx][ny].dist = ndist;
    }

    int get_Width(){
        return width;
    }

private:
    std::vector<std::tuple<int, int>> list_neighbors{{-1,-1}, {0, -1}, {1, -1}, {-1, 0}, {1,0}, {-1,1}, {0,1}, {-1,1}};
    std::tuple<int,int> transformar(int x, int z){
        int nx = (x/tile) + (width/tile)/2;
        int ny = (z/tile) + (width/tile)/2;
        return std::make_tuple(nx, ny);
    }

};


#endif //GOTOXY_GRID_H
