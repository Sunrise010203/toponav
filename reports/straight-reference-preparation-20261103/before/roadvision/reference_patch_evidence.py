"""S2 reference-only specialization of the frozen native appearance check.

All four competitors are identical when image, point and support mask are
the same. Evaluate that search once, without replacing measured NCC by 1,
changing gates or retaining any cache across calls. Target-frame evidence
must continue to use the original bidirectional appearance implementation.
"""
from roadvision.rgb_motion_checks import CHECK_POLICY, _ncc
from roadvision.rgb_resampling_checks import _supported_patch, _masked_alternative


def reference_appearance_evidence(image, point, mask):
    """Exactly ``appearance_evidence(image,image,point,point,mask,mask)``.

    Preserve the original diagnostic schema, including four logical search
    counts and the all-None scores on unavailable/invalid descriptors.
    This is only a necessary reference-texture check, not clearance evidence.
    """
    patch = _supported_patch(image, point, mask)
    result = dict(accepted=False, observed_ncc=None, alternative_ncc=None,
                  reverse_alternative_ncc=None, self_alternative_ncc=None, margin=None)
    if patch is None:
        return dict(result, reason='patch_boundary_flat_or_padding')
    observed = _ncc(patch, patch)
    alternative, count = _masked_alternative(patch, image, point, mask)
    result['alternative_search_counts'] = [count, count, count, count]
    if observed is None or alternative is None:
        return dict(result, reason='invalid_descriptor_search')
    margin = observed - alternative
    result.update(observed_ncc=observed, alternative_ncc=alternative,
                  reverse_alternative_ncc=alternative,
                  self_alternative_ncc=alternative, margin=margin)
    failures = []
    if observed < CHECK_POLICY['minimum_ncc']:
        failures.append('appearance_changed')
    if margin < CHECK_POLICY['minimum_margin']:
        failures.append('competing_match')
    if alternative >= CHECK_POLICY['self_similarity_max']:
        failures.append('self_repetition')
    return dict(result, accepted=not failures,
                reason='+'.join(failures) if failures else 'unique_patch_support')
