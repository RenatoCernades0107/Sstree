# SStree

## Correr el ejecutable ss_tree_indexing
### En el indexing.cpp se encuentra 
`

    const H5std_string FILE_NAME("embbeding.hdf5");
    std::vector<ImageData> data = readEmbeddingsFromHDF5(FILE_NAME);

    SsTree tree;
    tree.loadFromFile(filename);
    tree.test();
    tree.print();

    test_knn(tree, data);
`

donde se muestra el radio, si el tree es valido 
y finalmente un test para el knn donde busca el vecino más cercano
con búsqueda secuencial y usando el Sstree.