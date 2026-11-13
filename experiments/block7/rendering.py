"""Declared colour-albedo/exposure variants of the existing RGB renderer.

These are limited synthetic appearance factors, not realistic material or
physical lighting generalization. No geometry, sensor count or model changes.
"""
from roadvision.scene import World


class RenderedWorld(World):
    appearance_variant = 'nominal'

    def capture(self, position, heading, exposure=1.):
        if self.appearance_variant == 'material_albedo':
            import pybullet as p
            for body in [*self.bodies, *self.temporary]:
                p.changeVisualShape(body, -1, rgbaColor=(.24, .44, .72, 1), physicsClientId=self.client)
            p.changeVisualShape(self.floor, -1, rgbaColor=(.40, .24, .18, 1), physicsClientId=self.client)
        elif self.appearance_variant == 'lighting_exposure':
            exposure *= .70
        elif self.appearance_variant != 'nominal':
            raise ValueError('Unknown declared appearance factor')
        return super().capture(position, heading, exposure)
