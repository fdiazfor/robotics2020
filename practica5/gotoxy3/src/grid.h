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
                    array[k][l] = Value{false, nullptr, i, j};
                }
            }
        };

        struct Value
        {
            bool occupied = false;
            QGraphicsItem * paint_cell = nullptr;
            int cx, cy;
            //Añadir elemento gráfico para pintar texto
            //Crear en el constructor e inicializar a comillas comillas
            int dist = 0; //dist vecinos
        };

        std::vector<std::vector<Value>> array;
//Hacer método que me de lso vecinos de una celda que te de vecinos libres()que no están ocupadosos
//El metodo devuelve una lista con los vecinos que sean libres y que no estén en la lista 1.
//hacer un metodo con while que se llama desde donde se hace el click
        void create_graphic_items(QGraphicsScene &scene)
        {
            for (auto &row : array)
                for (auto &elem : row)
                {
                    elem.paint_cell = scene.addRect(-tile / 2, -tile / 2, tile, tile, QPen(QColor("Black")),
                                                    QBrush(QColor("Green")));
                    elem.paint_cell->setPos(elem.cx, elem.cy);
                }
        }

        void
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
        Qbrush cellColor(QColor("Red"),NULL);
        array[nx][ny].put_cell->setBrush(cellCcolor,Qt::SolidPattern);
    }

    void set_Dist(int x, int y, int ndist){
        auto[nx, ny] = this->transformar(x, y);
        this->array[nx][ny].dist = ndist;
    }

    int get_Width(){
        return width;
    }

private:
    std::tuple<int,int> transformar(int x, int z){
        int nx = (x/tile) + (width/tile)/2;
        int ny = (z/tile) + (width/tile)/2;
        return std::make_tuple(nx, ny);
    }

};


#endif //GOTOXY_GRID_H
