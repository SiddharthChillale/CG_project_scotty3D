
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>
#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;

    HalfedgeRef h0 = v->_halfedge;

    HalfedgeRef h_temp = h0;

    std::vector<HalfedgeRef> outerh;
    std::vector<FaceRef> fh;
    std::vector<EdgeRef> eh;
    std::vector<HalfedgeRef> innerh;

    do{
        outerh.push_back(h_temp->_next);
        innerh.push_back(h_temp);
        fh.push_back(h_temp->_face);
        eh.push_back(h_temp->_edge);

        h_temp = h_temp->_next->_next->_twin;

        

    } while(h_temp != h0);

    FaceRef n_face = new_face();
    for(int i = 0; i < outerh.size()-1; i++) {
        
        outerh[i]->_next = outerh[i + 1];
        outerh[i]->_vertex->_halfedge = outerh[i];
        outerh[i]->_face = n_face;
    }
    outerh[outerh.size() - 1]->_next = outerh[0];
    outerh[outerh.size() - 1]->_vertex->_halfedge = outerh[outerh.size() - 1];
    outerh[outerh.size() - 1]->_face = n_face;
    
    
    n_face->_halfedge = outerh[0];


    for(auto x : innerh) {
        erase(x);
        erase(x->_twin);
    }

    for(auto x : fh) {
        erase(x);
    }

    for(auto x : eh) {
        erase(x);
    }

    erase(v);

    return n_face;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;

    HalfedgeRef h0 = e->_halfedge;
    HalfedgeRef h1 = h0->_next;
    HalfedgeRef h2 = h1->_next;
    HalfedgeRef h3 = h0->_twin;
    HalfedgeRef h4 = h3->_next;
    HalfedgeRef h5 = h4->_next;

    h5->_next = h1;
    h2->_next = h4;
    h0->_vertex->_halfedge = h4;
    h3->_vertex->_halfedge = h1;

    h0->_face->_halfedge = h1;
    h5->_face = h1->_face;
    h4->_face = h1->_face;

    erase(h3->_face);
    erase(h0);
    erase(h3);
    erase(e);


    return h1->_face;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
Halfedge_Mesh::VertexRef Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    //if(!e->on_boundary()) {

        VertexRef v3 = e->_halfedge->_vertex;
        VertexRef v2 = e->_halfedge->_twin->_vertex;

        FaceRef f0 = e->_halfedge->_face;
        FaceRef f3 = e->_halfedge->_twin->_face;

        HalfedgeRef h0 = e->_halfedge;
        HalfedgeRef h1 = h0->_next;
        HalfedgeRef h2 = h1->_next;
        HalfedgeRef h3 = h0->_twin;
        HalfedgeRef h4 = h3->_next;
        HalfedgeRef h5 = h4->_next;

        HalfedgeRef h6 = h5->_twin;
        HalfedgeRef h7 = h4->_twin;
        HalfedgeRef h8 = h2->_twin;
        HalfedgeRef h9 = h1->_twin;

        EdgeRef e1 = h6->_edge;
        EdgeRef e2 = h9->_edge;
        EdgeRef e3 = h8->_edge;
        EdgeRef e4 = h7->_edge;

        std::vector<HalfedgeRef> outv3;
        std::vector<HalfedgeRef> outv3_twins;

        HalfedgeRef h_temp = h8;

        while(h_temp->_next->_next->_twin != h8) {

            if(h_temp->_face != f0 && h_temp->_face != f3) {
                outv3.push_back(h_temp);
            }
            if(h_temp->_twin->_face != f0 && h_temp->_twin->_face != f3) {
                outv3_twins.push_back(h_temp->_twin);
            }

            h_temp = h_temp->_next->_next->_twin;
        }

        for(auto x : outv3) {
            x->set_neighbors(x->_next, x->_twin, v2, x->_edge, x->_face);
        }

        h6->set_neighbors(h6->_next, h7, h6->_vertex, e1, h6->_face);
        h7->set_neighbors(h7->_next, h6, h7->_vertex, e1, h7->_face);
        h8->set_neighbors(h8->_next, h9, v2, e2, h8->_face);
        h9->set_neighbors(h9->_next, h8, h9->_vertex, e2, h9->_face);

        h7->_vertex->_halfedge = h7;
        h9->_vertex->_halfedge = h9;
        h6->_vertex->_halfedge = h6;

        e1->_halfedge = h6;
        e2->_halfedge = h9;
        e3->_halfedge = h8;
        e4->_halfedge = h7;

        v2->pos = e->center();

        erase(h0);
        erase(h1);
        erase(h2);
        erase(h3);
        erase(h4);
        erase(h5);

        erase(v3);

        erase(e);
        erase(e3);
        erase(e4);

        erase(f0);
        erase(f3);

        return v2;
    //}

    /*else {

        HalfedgeRef h0 = e->_halfedge;
        HalfedgeRef h1 = h0->_next;
        if(h1->_edge->on_boundary()) {

            HalfedgeRef h1 = h1->_next;
        }

        VertexRef v_edge = h1->_vertex;

        h1->set_neighbors(nullptr, h1->_twin, h1->_vertex, h1->_edge, NULL);

        erase(h0)

        return v_edge;

        
    }*/
}



/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;

    HalfedgeRef h0 = f->_halfedge;

    VertexRef v2 = h0->_next->_next->_vertex;
    
    EdgeRef e = h0->_edge;
    auto vf = collapse_edge(e) ;
    

    HalfedgeRef h_new = vf->_halfedge->_twin;

    while(h_new->_vertex != vf) {
        h_new = h_new->_twin->_next->_twin;
    }

    EdgeRef e_new = h_new->_edge;

    auto v_final = collapse_edge(e_new) ;
    

    v_final->pos = f->center();

    erase(f);

    return v_final;

}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;

    if(!e->on_boundary()) {

        HalfedgeRef Hedge = e->halfedge();
        HalfedgeRef HedgeTwin = Hedge->_twin;
        HalfedgeRef h5, h2, h1, h4;

        h1 = Hedge->_next;
        h4 = HedgeTwin->_next;

        // update the _halfedge pointed by their origin vertex to new ones
        Hedge->_vertex->_halfedge = HedgeTwin->_next;
        HedgeTwin->_vertex->_halfedge = Hedge->_next;

        // update the vertex of the halfedge
        VertexRef V0 = Hedge->_vertex;
        VertexRef V1 = HedgeTwin->_vertex;

        HalfedgeRef h_temp = Hedge->_next;

        while(h_temp->_next->_twin->_vertex != V0) {
            h_temp = h_temp->_next;
        }
        h2 = h_temp->_next;
        h_temp->_next = Hedge; // update the next of the halfedge that points to the new halfedge
        Hedge->_vertex = h_temp->_twin->_vertex;

        h_temp = HedgeTwin->_next;
        while(h_temp->_next->_twin->_vertex != V1) {
            h_temp = h_temp->_next;
        }
        h5 = h_temp->_next;
        h_temp->_next =
            HedgeTwin; // update the next of the halfedge that points to the new halfedge
        HedgeTwin->_vertex = h_temp->_twin->_vertex;

        // update the next of the halfedge
        Hedge->_next = h5;
        HedgeTwin->_next = h2;
        h5->_next = h1;

        h2->_next = h4;

        // update the _haledges pointed by the faces of the halfedge
        h2->_face = h4->_face;
        h5->_face = h1->_face;
        Hedge->_face->_halfedge = h1;
        HedgeTwin->_face->_halfedge = h4;

        return e;
    } else {
        return std::nullopt;
    }
    
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
Halfedge_Mesh::VertexRef Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    

        HalfedgeRef h0 = e->_halfedge;
        HalfedgeRef h3 = h0->_twin;
        HalfedgeRef h1 = h0->_next;
        HalfedgeRef h2 = h1->_next;
        HalfedgeRef h4 = h3->_next;
        HalfedgeRef h5 = h4->_next;

        VertexRef V0 = h0->_vertex;
        VertexRef V1 = h3->_vertex;
        VertexRef V2 = h5->_vertex;
        VertexRef V3 = h2->_vertex;

        VertexRef new_vtx = new_vertex();

        new_vtx->pos = e->center();
        new_vtx->_halfedge = h0;

        EdgeRef e_left_new = new_edge();
        EdgeRef e_right_new = new_edge();
        EdgeRef e0_new = new_edge();

        FaceRef f_left_new = new_face();
        FaceRef f_right_new = new_face();

        HalfedgeRef h_left_new = new_halfedge();
        HalfedgeRef h_right_new = new_halfedge();
        HalfedgeRef h0_new = new_halfedge();

        h0_new->_next = h_left_new;
        h0_new->_twin = new_halfedge();
        h0_new->_twin->_next = h3->_next;
        // twin of twin
        h0_new->_twin->_twin = h0_new;
        h0_new->_twin->_vertex = new_vtx;
        // twin edge
        h0_new->_twin->_edge = e0_new;
        h0_new->_twin->_face = f_right_new;
        h0_new->_vertex = V0;
        V0->_halfedge = h4;
        h0_new->_edge = e0_new;
        h0_new->_face = f_left_new;

        h0->_vertex = new_vtx;

        h2->_next = h0_new;

        // HalfedgeRef h_left_new = new_halfedge();

        h_left_new->_next = h2;
        h_left_new->_twin = new_halfedge();
        h_left_new->_twin->_next = h0;
        h_left_new->_twin->_twin = h_left_new;
        h_left_new->_twin->_vertex = h1->_twin->_vertex;
        h_left_new->_twin->_edge = e_left_new;
        h_left_new->_twin->_face = h0->_face;

        h_left_new->_vertex = new_vtx;
        h_left_new->_edge = e_left_new;
        h_left_new->_face = f_left_new;

        h1->_next = h_left_new->_twin;

        // HalfedgeRef h_right_new = new_halfedge();
        h_right_new->_next = h5;
        h_right_new->_twin = new_halfedge();
        h_right_new->_twin->_next = h0_new->_twin;
        h_right_new->_twin->_twin = h_right_new;
        h_right_new->_twin->_vertex = h5->_vertex;
        h_right_new->_twin->_edge = e_right_new;
        h_right_new->_twin->_face = f_right_new;
        // h_right_new->set_neighbors(//next, twin, vertex, edge, face)
        h_right_new->_vertex = new_vtx;
        h_right_new->_edge = e_right_new;
        h_right_new->_face = h3->_face;

        h4->_next = h_right_new->_twin;

        h3->_next = h_right_new;

        h0->_face->_halfedge = h0;
        h3->_face->_halfedge = h3;
        h1->_face = h0->_face;
        h5->_face = h3->_face;
        h2->_face = f_left_new;
        h4->_face = f_right_new;

        f_left_new->_halfedge = h2;
        f_right_new->_halfedge = h4;

        e_left_new->_halfedge = h_left_new;
        e_right_new->_halfedge = h_right_new;
        e0_new->_halfedge = h0_new;

        // setting is_new of 3 new edges and the newly inserted vertex to true
        new_vtx->is_new = true;
        e_left_new->is_new = true;
        e_left_new->is_part_of_old = false;

        e_right_new->is_new = true;
        e_right_new->is_part_of_old = false;

        e0_new->is_new = true;
        e->is_new = true;
        
        //new_vtx->new_pos = (3 / 8) * V0->pos + (3 / 8) * V1->pos + (1 / 8) * V2->pos + (1 / 8) * V3->pos;

        //new_vtx->new_pos = (V0->pos + V1->pos ) /2 ;


        return new_vtx;
    
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for bevel_vertex, it has one element, the original vertex position,
    for bevel_edge, two for the two vertices, and for bevel_face, it has the original
    position of each vertex in order starting from face->halfedge. You should use these 
    positions, as well as the normal and tangent offset fields to assign positions to 
    the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    only responsible for updating the *connectivity* of the mesh---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions. These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)f;
    return std::nullopt;
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions in start_positions. So, you can write 
    loops of the form:

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
    (void)normal_offset;
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...

    
    for(auto f_iter = faces.begin(); f_iter != faces.end(); f_iter++) {
        
        
        
        HalfedgeRef h0 = f_iter->halfedge();
        
        while(h0->next()->next()->next() != h0) {
            
            VertexRef v0 = h0->vertex();
            VertexRef v1 = h0->next()->next()->vertex();

            HalfedgeRef h_temp = h0;
            while(h_temp->next() != h0) {
                h_temp = h_temp->next();
            }

            HalfedgeRef h_new = new_halfedge();
            HalfedgeRef h_new_twin = new_halfedge();
            FaceRef f_new = new_face();

            EdgeRef e_new = new_edge();

            h_new->set_neighbors(h0, h_new_twin, v1, e_new, f_new);
            h_new->twin()->set_neighbors(h0->next()->next(), h_new, v0, e_new, f_iter);

            h0->next()->next() = h_new;

            e_new->_halfedge = h_new;

            f_iter->_halfedge = h_new_twin;
            f_new->_halfedge = h_new;

            h0->_face = f_new;
            h0->_next->_face = f_new;

            h_temp->_next = h_new_twin;
            
            h0 = h_new_twin;
        }

        
    }



}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided) mesh. They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos. The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces

    // Edges

    // Vertices
}

/*
    This routine should increase the number of triangles in the mesh
    using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Each vertex and edge of the original mesh can be associated with a
    // vertex in the new (subdivided) mesh.
    // Therefore, our strategy for computing the subdivided vertex locations is to
    // *first* compute the new positions
    // using the connectivity of the original (coarse) mesh. Navigating this mesh
    // will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse.  We
    // will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.
    
    // Compute new positions for all the vertices in the input mesh using
    // the Loop subdivision rule and store them in Vertex::new_pos.
    //    At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    
    // Next, compute the subdivided vertex positions associated with edges, and
    // store them in Edge::new_pos.
    
    // Next, we're going to split every edge in the mesh, in any order.
    // We're also going to distinguish subdivided edges that came from splitting 
    // an edge in the original mesh from new edges by setting the boolean Edge::is_new. 
    // Note that in this loop, we only want to iterate over edges of the original mesh.
    // Otherwise, we'll end up splitting edges that we just split (and the
    // loop will never end!)
    
    // Now flip any new edge that connects an old and new vertex.
    
    // Finally, copy new vertex positions into the Vertex::pos.

    for(auto x = vertices.begin(); x != vertices.end(); x++) {
        x->is_new = false;

        float n = (float)(x->degree());
        float u;

        HalfedgeRef h0 = x->halfedge();
        std::vector<VertexRef> outerv;
        // outerv.push_back(h0->next()->vertex());

        for(HalfedgeRef h_temp = h0->next()->next()->twin(); h_temp != h0;
            h_temp = h_temp->next()->next()->twin()) {
            outerv.push_back(h_temp->next()->vertex());
        }

        assert(n >= 3);
        
        u = (n == 3.0f) ? (3.0f / 16.0f) : (3.0f / (8.0f * n));

        // the code below throws the warning of scope of u as an error 
        /*if(n == 3.0f ) {
            u = (3 / 16);
        } 
        else if(n > 3.0f ) {
            u = (3 / (8 * n));
        }*/


        x->new_pos = (1.0f - n * u) * (x->pos); /* + (u) * positions of surrounding vertices*/

        
        Vec3 outervsum_pos(0, 0, 0);
        outervsum_pos = (u) * (h0->next()->vertex()->pos);

        for(auto y : outerv) {
            outervsum_pos = outervsum_pos + (u)*y->pos;
        }

        x->new_pos = x->new_pos + outervsum_pos;
        
    }

    for(auto x = edges.begin(); x != edges.end(); x++) {
        x->is_new = false;
    }


    int n_edges = (int)edges.size();

    EdgeRef e_iter = edges.begin();


    for (int i = 0; i<n_edges ; i++) {
        EdgeRef e_next = e_iter;
        e_next++;

        if(!e_iter->is_new) {
           
           split_edge(e_iter);
        }

        e_iter = e_next;
        
    }



    

    for(auto x = edges.begin(); x != edges.end(); x++) {
        if(!x->is_part_of_old) {

            VertexRef v0 = x->halfedge()->vertex();
            VertexRef v1 = x->halfedge()->twin()->vertex();

            if(!(v1->is_new && v0->is_new)) {
                flip_edge(x);
            }
        }
    }

    


    for(auto m = vertices.begin(); m != vertices.end(); m++) {
        if(m->is_new) {

            HalfedgeRef h0 = m->halfedge();
            
            HalfedgeRef h1 = h0->twin();
            HalfedgeRef h2 = h1->next()->twin()->next()->next()->twin()->next()->next();
            HalfedgeRef h3 = h1->next()->twin()->next()->twin()->next()->next();
            HalfedgeRef h4 = h0->next()->next()->twin()->next()->twin()->next()->next();

            VertexRef v1 = h1->vertex();
            VertexRef v2 = h2->vertex();
            VertexRef v3 = h3->vertex();
            VertexRef v4 = h4->vertex();

            m->new_pos = (3.0f / 8.0f) * (v1->pos + v3->pos) + (1.0f / 8.0f) * (v2->pos + v4->pos);


            
        }
    }

    for(auto m = vertices.begin(); m != vertices.end(); m++) {
    
        m->pos = m->new_pos;

    }


}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate is called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
