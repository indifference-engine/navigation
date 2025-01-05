#include <stdio.h>
#include "../../src/sliding_navigation_collision.h"
#include "../../src/sliding_navigation_collision_result.h"

static int exit_code = 0;

static void check_exact(
    const char *const description_a,
    const char *const description_b,
    const float expected,
    const float actual)
{
  if (actual != expected)
  {
    printf("FAIL %s %s expected %f actual %f\n", description_a, description_b, expected, actual);
    exit_code = 1;
  }
}

static void check_approximate(
    const char *const description_a,
    const char *const description_b,
    const float expected,
    const float actual)
{
  if (actual != actual || expected < actual - 0.0025f || expected > actual + 0.0025f)
  {
    printf("FAIL %s %s expected %f actual %f\n", description_a, description_b, expected, actual);
    exit_code = 1;
  }
}

static void check_int(
    const char *const description_a,
    const char *const description_b,
    const int expected,
    const int actual)
{
  if (actual != expected)
  {
    printf("FAIL %s %s expected %d actual %d\n", description_a, description_b, expected, actual);
    exit_code = 1;
  }
}

static const int face_vertex_counts[] = {3, 5, 4, 6};

static const int face_vertex_offsets[] = {0, 3, 8, 12};

static const float face_normals[] = {
    0.07895775884389877f,
    0.49244746565818787f,
    0.8667532205581665f,
    -0.7232892513275146f,
    0.2542363703250885f,
    0.642041027545929f,
    0.09298569709062576f,
    0.7767484784126282f,
    0.6229087114334106f,
    -0.12398175895214081f,
    -0.7511836290359497f,
    0.6483453512191772f,
};

static const float face_vertex_locations[] = {
    -3.4020490646362305f,
    2.8101935386657715f,
    -0.29557961225509644f,
    -4.021132946014404f,
    0.4352002739906311f,
    1.110173225402832f,
    -5.086987018585205f,
    1.94649076461792f,
    0.3486257791519165f,
    -1.8411436080932617f,
    -2.8504514694213867f,
    0.7629717588424683f,
    -1.3159351348876953f,
    -1.4987578392028809f,
    0.8193982839584351f,
    -0.7699069976806641f,
    -1.1729934215545654f,
    1.3055280447006226f,
    -0.3965773284435272f,
    -3.695011615753174f,
    2.724773406982422f,
    -1.7895712852478027f,
    -3.858870029449463f,
    1.2203859090805054f,
    -0.4381864070892334f,
    0.2253333330154419f,
    2.216573715209961f,
    -1.7317837476730347f,
    0.9585509300231934f,
    1.4953770637512207f,
    -0.9570263028144836f,
    1.5087556838989258f,
    0.6936352252960205f,
    1.082206130027771f,
    0.8231964111328125f,
    1.2440972328186035f,
    3.376988410949707f,
    3.536540985107422f,
    1.8753199577331543f,
    4.3473639488220215f,
    2.9355649948120117f,
    1.3645833730697632f,
    2.634784698486328f,
    2.1809427738189697f,
    0.16277146339416504f,
    1.4529716968536377f,
    2.790022134780884f,
    0.6424660086631775f,
    1.3432226181030273f,
    3.4920597076416016f,
    1.4348706007003784f,
    2.3029632568359375f,
    3.920729398727417f,
    2.115065097808838f,
};

static const float edge_exit_normals[] = {
    0.8445626497268677f,
    -0.41813111305236816f,
    -0.33448511362075806f,
    -0.7482595443725586f,
    -0.6309695839881897f,
    -0.20490238070487976f,
    -0.5380623936653137f,
    0.6443790197372437f,
    -0.5433824062347412f,
    -0.3881560266017914f,
    0.18847911059856415f,
    -0.9021145105361938f,
    0.2913234829902649f,
    0.6106106042861938f,
    -0.7364001274108887f,
    0.8732513189315796f,
    0.33066919445991516f,
    0.35789668560028076f,
    0.28668972849845886f,
    -0.9441192150115967f,
    -0.16262786090373993f,
    -0.3236362040042877f,
    -0.4045316278934479f,
    -0.855344295501709f,
    -0.5978249311447144f,
    -0.3412596583366394f,
    0.7253599762916565f,
    -0.7264787554740906f,
    0.6329079270362854f,
    -0.2676868736743927f,
    0.2879942059516907f,
    0.09123139083385468f,
    -0.9532766342163086f,
    0.4545976519584656f,
    -0.8736056685447693f,
    0.17364968359470367f,
    0.6291472911834717f,
    0.6354882121086121f,
    0.44758081436157227f,
    0.5895053148269653f,
    -0.7023043036460876f,
    -0.39906418323516846f,
    -0.5148865580558777f,
    -0.4032585620880127f,
    -0.7564880847930908f,
    -0.9757379293441772f,
    -0.21241706609725952f,
    0.053051725029945374f,
    -0.5568119883537292f,
    0.7758399844169617f,
    0.29670313000679016f,
    0.36428022384643555f,
    0.5488821864128113f,
    0.7523483633995056f,
};

static const int face_edge_neighbor_counts[] = {
    1, 0, 2,
    3, 0, 0, 1, 2,
    0, 0, 2, 1,
    5, 4, 3, 1, 2, 3};

static void
scenario(
    const char *const description,
    const float from_x, const float from_y, const float from_z,
    const int face_index,
    const float to_x, const float to_y, const float to_z,
    const float result_from_x, const float result_from_y, const float result_from_z,
    const float result_to_x, const float result_to_y, const float result_to_z,
    const int result)
{
  const float different_from[] = {from_x, from_y, from_z};
  const float different_to[] = {to_x, to_y, to_z};
  float different_result_from[] = {1.0f, 2.0f, 3.0f};
  float different_result_to[] = {4.0f, 5.0f, 6.0f};

  const int different_result = sliding_navigation_collision(different_from, face_index, different_to, face_vertex_counts, face_vertex_offsets, face_vertex_locations, face_normals, edge_exit_normals, face_edge_neighbor_counts, different_result_from, different_result_to);

  check_exact(description, "different from x", from_x, different_from[0]);
  check_exact(description, "different from y", from_y, different_from[1]);
  check_exact(description, "different from z", from_z, different_from[2]);
  check_exact(description, "different to x", to_x, different_to[0]);
  check_exact(description, "different to y", to_y, different_to[1]);
  check_exact(description, "different to z", to_z, different_to[2]);
  check_approximate(description, "different result from x", result_from_x, different_result_from[0]);
  check_approximate(description, "different result from y", result_from_y, different_result_from[1]);
  check_approximate(description, "different result from z", result_from_z, different_result_from[2]);
  check_approximate(description, "different result to x", result_to_x, different_result_to[0]);
  check_approximate(description, "different result to y", result_to_y, different_result_to[1]);
  check_approximate(description, "different result to z", result_to_z, different_result_to[2]);
  check_int(description, "different result", result, different_result);

  float same_from[] = {from_x, from_y, from_z};
  float same_to[] = {to_x, to_y, to_z};

  const int same_result = sliding_navigation_collision(same_from, face_index, same_to, face_vertex_counts, face_vertex_offsets, face_vertex_locations, face_normals, edge_exit_normals, face_edge_neighbor_counts, same_from, same_to);

  check_approximate(description, "same from x", result_from_x, same_from[0]);
  check_approximate(description, "same from y", result_from_y, same_from[1]);
  check_approximate(description, "same from z", result_from_z, same_from[2]);
  check_approximate(description, "same to x", result_to_x, same_to[0]);
  check_approximate(description, "same to y", result_to_y, same_to[1]);
  check_approximate(description, "same to z", result_to_z, same_to[2]);
  check_int(description, "same result", result, same_result);
}

int main(const int argc, const char *const *const argv)
{
  (void)(argc);
  (void)(argv);

  scenario(
      "no collision",
      -0.6593204736709595f, 1.7540239095687866f, 1.7363266944885254f,
      2,
      -0.7968413233757019f, 0.8086849451065063f, 1.6531301736831665f,
      -0.6593204736709595f, 1.7540239095687866f, 1.7363266944885254f,
      -0.7968413233757019f, 0.8086849451065063f, 1.6531301736831665f,
      SLIDING_NAVIGATION_COLLISION_RESULT_NONE);

  scenario(
      "surface collision",
      -0.7353870272636414f, 1.3286099433898926f, 1.4362154006958008f,
      2,
      -0.4760225713253021f, 0.9154407978057861f, 1.1747195720672607f,
      -0.5417287945747375f, 1.020111322402954f, 1.240965723991394f,
      -0.465194f, 1.005900f, 1.247262f,
      SLIDING_NAVIGATION_COLLISION_RESULT_SURFACE);

  scenario(
      "edge collision",
      -0.799696683883667f, 0.8778424859046936f, 1.7735588550567627f,
      2,
      -1.7022922039031982f, 1.066243052482605f, 2.0128259658813477f,
      -1.2558252811431885f, 0.9730511903762817f, 1.8944730758666992f,
      -1.510417f, 1.175772f, 1.780018f,
      SLIDING_NAVIGATION_COLLISION_RESULT_EDGE);

  scenario(
      "edge pass",
      0.0352475643157959f, 1.1076184511184692f, 1.4057743549346924f,
      2,
      0.20977041125297546f, 1.3069931268692017f, 0.8717013597488403f,
      0.16508668661117554f, 1.2559465169906616f, 1.0084420442581177f,
      0.20977041125297546f, 1.3069931268692017f, 0.8717013597488403f,
      2);

  scenario(
      "surface collision before edge pass",
      0.44600802659988403f, 0.7854040265083313f, 1.6033046245574951f,
      2,
      0.6767760515213013f, 0.33395862579345703f, 1.5399271249771118f,
      0.5306594371795654f, 0.6198027729988098f, 1.5800561904907227f,
      0.698483f, 0.515282f, 1.685338f,
      SLIDING_NAVIGATION_COLLISION_RESULT_SURFACE);

  scenario(
      "edge pass before surface collision",
      0.44600802659988403f, 0.7854040265083313f, 1.6033046245574951f,
      2,
      0.678392767906189f, 0.36975809931755066f, 1.5947304964065552f,
      0.5424019694328308f, 0.6129928231239319f, 1.599748134613037f,
      0.678392767906189f, 0.36975809931755066f, 1.5947304964065552f,
      3);

  scenario(
      "surface collision before edge collision",
      -0.9451434016227722f, 1.466844081878662f, 1.2895077466964722f,
      2,
      -1.2491717338562012f, 1.3015402555465698f, 0.6656430959701538f,
      -1.1345611810684204f, 1.3638553619384766f, 0.9008233547210693f,
      -1.230058f, 1.461206f, 0.793686f,
      SLIDING_NAVIGATION_COLLISION_RESULT_SURFACE);

  scenario(
      "edge collision before surface collision",
      -0.9451434016227722f, 1.466844081878662f, 1.2895077466964722f,
      2,
      -1.8238425254821777f, 1.1546778678894043f, -0.2062366008758545f,
      -1.1484935283660889f, 1.3946020603179932f, 0.9433599710464478f,
      -1.354168f, 0.745497f, -0.033175f,
      SLIDING_NAVIGATION_COLLISION_RESULT_EDGE);

  scenario(
      "edge collision before edge collision",
      -1.2517286539077759f, 1.153049111366272f, 1.749252438545227f,
      2,
      -2.138270139694214f, 1.5075002908706665f, 1.9872474670410156f,
      -1.5096309185028076f, 1.2561619281768799f, 1.8184871673583984f,
      -1.891693f, 1.648255f, 1.688068f,
      SLIDING_NAVIGATION_COLLISION_RESULT_EDGE);

  scenario(
      "edge collision after edge collision",
      -1.2517286539077759f, 1.153049111366272f, 1.749252438545227f,
      2,
      -2.094743251800537f, 1.6770308017730713f, 1.9091947078704834f,
      -1.5263663530349731f, 1.3237521648406982f, 1.8013584613800049f,
      -1.653305f, 1.292450f, 2.071852f,
      SLIDING_NAVIGATION_COLLISION_RESULT_EDGE);

  scenario(
      "edge collision before edge pass",
      -0.8459683060646057f, 1.3359043598175049f, 0.9404218792915344f,
      2,
      -1.355805516242981f, 1.9333240985870361f, 0.33385685086250305f,
      -0.9893378019332886f, 1.5039026737213135f, 0.7698519229888916f,
      -0.880162f, 1.518943f, 0.509118f,
      SLIDING_NAVIGATION_COLLISION_RESULT_EDGE);

  scenario(
      "edge pass before edge collision",
      -0.8459683060646057f, 1.3359043598175049f, 0.9404218792915344f,
      2,
      -0.7945818305015564f, 2.199634075164795f, 0.013435125350952148f,
      -0.8344507813453674f, 1.529496669769287f, 0.7326514720916748f,
      -0.7945818305015564f, 2.199634075164795f, 0.013435125350952148f,
      2);

  scenario(
      "edge pass before edge pass",
      0.6804202198982239f, 0.9724671840667725f, 1.3651516437530518f,
      2,
      2.4518818855285645f, 0.9358981847763062f, 0.9669672846794128f,
      1.1150693893432617f, 0.9634945392608643f, 1.2674524784088135f,
      2.4518818855285645f, 0.9358981847763062f, 0.9669672846794128f,
      2);

  scenario(
      "edge pass after edge pass",
      0.6804202198982239f, 0.9724671840667725f, 1.3651516437530518f,
      2,
      2.429912567138672f, 0.6535996198654175f, 1.4251762628555298f,
      1.1516081094741821f, 0.8865871429443359f, 1.3813179731369019f,
      2.429912567138672f, 0.6535996198654175f, 1.4251762628555298f,
      3);

  scenario(
      "alternative edge collision before edge pass",
      -0.4319901764392853f, 0.5731868743896484f, 2.097071647644043f,
      2,
      0.4020518362522125f, -0.49490365386009216f, 3.200631856918335f,
      -0.25350940227508545f, 0.34462088346481323f, 2.33322811126709f,
      0.4020518362522125f, -0.49490365386009216f, 3.200631856918335f,
      3);

  scenario(
      "alternative edge pass before edge collision",
      -0.4319901764392853f, 0.5731868743896484f, 2.097071647644043f,
      2,
      -0.6737288236618042f, -0.5296585559844971f, 3.494502067565918f,
      -0.4649297893047333f, 0.42291176319122314f, 2.287487268447876f,
      0.118640f, -0.077346f, 2.533096f,
      SLIDING_NAVIGATION_COLLISION_RESULT_EDGE);

  return exit_code;
}
