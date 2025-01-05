#include <stdio.h>
#include "../../src/constrain_to_navigable_surface.h"

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

static const int face_vertex_counts[] = {3, 5, 4, 6};

static const int face_vertex_offsets[] = {0, 3, 8, 12};

static const float face_normals[] = {
    0.07895775884389877f,
    0.49244746565818787f,
    0.8667532205581665f,
    -0.7232892513275146f,
    0.2542363703250885f,
    0.642041027545929f,
    -0.09298569709062576f,
    -0.7767484784126282f,
    -0.6229087114334106f,
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

static const float edge_normals[] = {
    0.9725521206855774f,
    -0.22895699739456177f,
    0.04148668050765991f,
    -0.8424692153930664f,
    -0.43185123801231384f,
    0.32210269570350647f,
    -0.5329273343086243f,
    0.7556453347206116f,
    -0.38077372312545776f,
    -0.5881147980690002f,
    0.2604789733886719f,
    -0.7656837701797485f,
    -0.10690337419509888f,
    0.8773258328437805f,
    -0.46783649921417236f,
    0.6785890460014343f,
    0.43394654989242554f,
    0.5926274657249451f,
    -0.13480409979820251f,
    -0.9638553857803345f,
    0.22980590164661407f,
    -0.6889761686325073f,
    -0.3283272981643677f,
    -0.6461523771286011f,
    -0.6153386831283569f,
    -0.44700878858566284f,
    0.6492623090744019f,
    -0.7765533328056335f,
    0.44812941551208496f,
    -0.4428826570510864f,
    0.3848402798175812f,
    0.5489599704742432f,
    -0.7419843673706055f,
    0.593180775642395f,
    -0.5456900596618652f,
    0.5919111967086792f,
    0.6184095144271851f,
    0.4524862766265869f,
    0.642515242099762f,
    0.6258806586265564f,
    -0.5662204027175903f,
    -0.5363466739654541f,
    -0.5343284010887146f,
    -0.5000267624855042f,
    -0.6815176010131836f,
    -0.986912190914154f,
    0.02545109950006008f,
    -0.15923726558685303f,
    -0.6300894021987915f,
    0.5643534660339355f,
    0.5333784818649292f,
    0.3682067394256592f,
    0.5719105005264282f,
    0.7330362200737f,
};

static const float edge_coefficients[] = {
    -0.07738548517227173f,
    -0.29687416553497314f,
    0.17571909725666046f,
    -0.26646363735198975f,
    0.3778228163719177f,
    -0.19038695096969604f,
    0.4212346076965332f,
    0.21592575311660767f,
    -0.16105137765407562f,
    0.24937447905540466f,
    0.6417982578277588f,
    0.026791905984282494f,
    0.8523813486099243f,
    0.5085370540618896f,
    0.7588765621185303f,
    0.04384785145521164f,
    -0.29621291160583496f,
    0.16669143736362457f,
    -0.3292769193649292f,
    -0.038732971996068954f,
    -0.3556082248687744f,
    -0.041969820857048035f,
    0.8206561803817749f,
    -0.3722459673881531f,
    -0.47364968061447144f,
    0.2684670388698578f,
    -0.26406559348106384f,
    0.5012131929397583f,
    0.355943500995636f,
    -0.5186702013015747f,
    0.4135141670703888f,
    -0.1390172392129898f,
    0.11162230372428894f,
    -0.42060860991477966f,
    -0.16539567708969116f,
    0.26903051137924194f,
    0.6205825805664062f,
    -0.3843410909175873f,
    -0.32663047313690186f,
    -0.34620407223701477f,
    -0.1525495946407318f,
    -0.24295058846473694f,
    -0.5915671586990356f,
    0.30488017201423645f,
    0.24011541903018951f,
    -0.09688244014978409f,
    0.6197329163551331f,
    0.6995055675506592f,
    0.6122652888298035f,
    0.2734692692756653f,
    0.4339292049407959f,
    0.790533185005188f,
    -0.2827807664871216f,
    -0.17646372318267822f,
};

static void scenario(
    const char *const description,
    const float unconstrained_x, const float unconstrained_y, const float unconstrained_z,
    const float constrained_x, const float constrained_y, const float constrained_z)
{
  const float different_unconstrained[] = {unconstrained_x, unconstrained_y, unconstrained_z};
  float different_constrained[] = {0.7468627737f, 0.6126531178f, 0.1742534262f};

  constrain_to_navigable_surface(different_unconstrained,
                                 face_vertex_counts,
                                 face_vertex_offsets,
                                 face_vertex_locations,
                                 face_normals,
                                 edge_normals,
                                 edge_coefficients,
                                 2, different_constrained);

  check_exact(description, "different unconstrained x", unconstrained_x, different_unconstrained[0]);
  check_exact(description, "different unconstrained y", unconstrained_y, different_unconstrained[1]);
  check_exact(description, "different unconstrained z", unconstrained_z, different_unconstrained[2]);

  check_approximate(description, "different constrained x", constrained_x, different_constrained[0]);
  check_approximate(description, "different constrained y", constrained_y, different_constrained[1]);
  check_approximate(description, "different constrained z", constrained_z, different_constrained[2]);

  float same[] = {unconstrained_x, unconstrained_y, unconstrained_z};
  constrain_to_navigable_surface(same,
                                 face_vertex_counts,
                                 face_vertex_offsets,
                                 face_vertex_locations,
                                 face_normals,
                                 edge_normals,
                                 edge_coefficients,
                                 2, same);
  check_approximate(description, "same x", constrained_x, same[0]);
  check_approximate(description, "same y", constrained_y, same[1]);
  check_approximate(description, "same z", constrained_z, same[2]);
}

int main(const int argc, const char *const *const argv)
{
  (void)(argc);
  (void)(argv);

  scenario(
      "under ad",
      -0.44776129722595215f, -0.4037471413612366f, 2.3634936809539795f, -0.4381864070892334f, 0.2253333330154419f, 2.216573715209961f);

  scenario(
      "above ad",
      -0.32799556851387024f, 0.11648856848478317f, 2.879356622695923f, -0.4381864070892334f, 0.2253333330154419f, 2.216573715209961f);

  scenario(
      "under bc",
      -2.1550962924957275f, 0.7870222926139832f, 1.5255401134490967f, -1.7317837476730347f, 0.9585509300231934f, 1.4953770637512207f);

  scenario(
      "above bc",
      -2.1393260955810547f, 1.4126778841018677f, 2.172755002975464f, -1.7317837476730347f, 0.9585509300231934f, 1.4953770637512207f);

  scenario(
      "under cd",
      1.5398374795913696f, 0.3737923502922058f, 1.0536272525787354f, 1.082206130027771f, 0.8231964111328125f, 1.2440972328186035f);

  scenario(
      "above cd",
      2.1589162349700928f, 1.1578724384307861f, 1.2380948066711426f, 1.082206130027771f, 0.8231964111328125f, 1.2440972328186035f);

  scenario(
      "under surface",
      -0.48655855655670166f, 0.0667107105255127f, 1.4121458530426025f, -0.42808979749679565f, 0.5551248788833618f, 1.8038263320922852f);

  scenario(
      "above surface",
      -0.3538905084133148f, 1.1749427318572998f, 2.3008854389190674f, -0.42808979749679565f, 0.5551248788833618f, 1.8038263320922852f);

  scenario(
      "under a",
      -1.4930438995361328f, -0.12190787494182587f, 1.7247934341430664f, -1.1319071054458618f, 0.6185377836227417f, 1.8298158645629883f);

  scenario(
      "above a",
      -1.2354415655136108f, 1.1127464771270752f, 2.517969846725464f, -1.1319071054458618f, 0.6185377836227417f, 1.8298158645629883f);

  scenario(
      "under b",
      -1.6809687614440918f, 1.383725881576538f, 0.20064657926559448f, -1.0745216608047485f, 1.4253146648406982f, 0.8152226209640503f);

  scenario(
      "above b",
      -1.3303054571151733f, 2.152769088745117f, 1.0672723054885864f, -1.0745216608047485f, 1.4253146648406982f, 0.8152226209640503f);

  scenario(
      "under d",
      1.015620231628418f, -0.06502214074134827f, 0.8374176621437073f, 0.9826131463050842f, 0.7840335965156555f, 1.307799220085144f);

  scenario(
      "above d",
      1.5372536182403564f, 0.4288828372955322f, 1.9565962553024292f, 0.9826131463050842f, 0.7840335965156555f, 1.307799220085144f);

  return exit_code;
}
