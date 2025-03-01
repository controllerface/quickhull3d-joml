package com.github.quickhull3d;

/*
 * #%L
 * A Robust 3D Convex Hull Algorithm in Java
 * %%
 * Copyright (C) 2004 - 2014 John E. Lloyd
 * %%
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * #L%
 */

import org.joml.Vector3f;
import org.junit.Assert;
import org.junit.Test;

public class TestConvexHullFloat
{

    @Test
    public void testConvexHull() throws Exception {
        Vector3f[] points = new Vector3f[]{
            new Vector3f(0.3215426810286406f, 0.1678336189760208f, -0.2203710966001927f),
            new Vector3f(0.2229772524190855f, -0.4213242506806965f, -0.1966818060695024f),
            new Vector3f(0.3688830163971363f, -0.1831502133823468f, -0.2056387967482571f),
            new Vector3f(-0.1712592515826777f, -0.3542439228428937f, 0.2223876390814666f),
            new Vector3f(-0.3309556113844324f, -0.370961861099081f, 0.2439994981922204f),
            new Vector3f(-0.1004397059794885f, -0.09014152417903909f, -0.008600084584765189f),
            new Vector3f(0.458374538420117f, -0.09914027349943322f, -0.2505798421339875f),
            new Vector3f(-0.4954086979808367f, -0.3339869997780649f, -0.3195065691317492f),
            new Vector3f(0.053091190339151f, 0.3036317017894533f, 0.1380056861210668f),
            new Vector3f(0.4615616439483703f, 0.4665423151725366f, 0.1766835406205464f),
            new Vector3f(-0.4797380864431505f, 0.0419809916447671f, -0.4254776681079321f),
            new Vector3f(-0.003168473023146823f, -0.2525299883005488f, -0.27151530400991f),
            new Vector3f(-0.3577162826971303f, -0.1375644040643837f, -0.04494194644032229f),
            new Vector3f(-0.3392973838740004f, 0.4288679723896719f, -0.01599531622230571f),
            new Vector3f(0.1667164640191164f, 0.003605551555385444f, -0.4014989499947977f),
            new Vector3f(0.00714666676441833f, 0.1140243407469469f, 0.407090128778564f),
            new Vector3f(-0.03621271768232132f, 0.3728502838619522f, 0.4947140370446388f),
            new Vector3f(-0.3411871756810576f, -0.3328629143842151f, -0.4270033635450559f),
            new Vector3f(0.3544683273457627f, -0.450828987127942f, -0.0827870439577727f),
            new Vector3f(-0.4018510635028137f, 0.08917494033386464f, -0.2367824197158054f),
            new Vector3f(0.3978697768392692f, -0.002667689232777493f, 0.1641431727112673f),
            new Vector3f(-0.245701439441835f, 0.495905311308713f, -0.3194406286994373f),
            new Vector3f(0.161352035739787f, -0.1563404972258401f, 0.3852604361113724f),
            new Vector3f(0.07214279572678994f, -0.4960366976410492f, 0.1112227161519441f),
            new Vector3f(0.3201855824516951f, 0.359077846965825f, 0.02136723140381946f),
            new Vector3f(0.1190541238701475f, -0.05734495917087884f, 0.2032677509852384f),
            new Vector3f(0.3210853052521919f, 0.4807189479290684f, 0.4433501688235907f),
            new Vector3f(0.3862800354941562f, 0.2085496142586224f, 0.09336129957191763f),
            new Vector3f(0.1233572616459404f, 0.265491605052251f, 0.117400122450106f),
            new Vector3f(0.1438531872293476f, -0.2594872752758556f, -0.2026374435076839f),
            new Vector3f(0.2724846394476338f, -0.3506708492996831f, 0.2750346518820475f),
            new Vector3f(-0.4926118841325975f, -0.3279366743079728f, 0.3683135596740186f),
            new Vector3f(0.2459906458351674f, 0.3647787136629026f, -0.1641662355178652f),
            new Vector3f(-0.141922976953837f, -0.2994764654892278f, -0.3009570467294725f),
            new Vector3f(-0.1850859398814719f, 0.2606059478228967f, 0.004159106876849283f),
            new Vector3f(-0.09789466634196664f, -0.3156603563722785f, -0.303610991503681f),
            new Vector3f(0.2100642609503719f, -0.4499717643018549f, 0.3245569875692548f),
            new Vector3f(-0.1707163766685095f, -0.2301452446078371f, -0.05112823569320907f),
            new Vector3f(-0.312260808713977f, -0.1674135249735914f, 0.2808831662692904f),
            new Vector3f(-0.1966306233747216f, 0.2291105671125563f, -0.3387042454804333f)
        };
        Vector3f[] expected = {
            new Vector3f(0.3215426810286406f, 0.1678336189760208f, -0.2203710966001927f),
            new Vector3f(0.2229772524190855f, -0.4213242506806965f, -0.1966818060695024f),
            new Vector3f(0.458374538420117f, -0.09914027349943322f, -0.2505798421339875f),
            new Vector3f(-0.4954086979808367f, -0.3339869997780649f, -0.3195065691317492f),
            new Vector3f(0.4615616439483703f, 0.4665423151725366f, 0.1766835406205464f),
            new Vector3f(-0.4797380864431505f, 0.0419809916447671f, -0.4254776681079321f),
            new Vector3f(-0.3392973838740004f, 0.4288679723896719f, -0.01599531622230571f),
            new Vector3f(0.1667164640191164f, 0.003605551555385444f, -0.4014989499947977f),
            new Vector3f(-0.03621271768232132f, 0.3728502838619522f, 0.4947140370446388f),
            new Vector3f(-0.3411871756810576f, -0.3328629143842151f, -0.4270033635450559f),
            new Vector3f(0.3544683273457627f, -0.450828987127942f, -0.0827870439577727f),
            new Vector3f(0.3978697768392692f, -0.002667689232777493f, 0.1641431727112673f),
            new Vector3f(-0.245701439441835f, 0.495905311308713f, -0.3194406286994373f),
            new Vector3f(0.161352035739787f, -0.1563404972258401f, 0.3852604361113724f),
            new Vector3f(0.07214279572678994f, -0.4960366976410492f, 0.1112227161519441f),
            new Vector3f(0.3210853052521919f, 0.4807189479290684f, 0.4433501688235907f),
            new Vector3f(0.2724846394476338f, -0.3506708492996831f, 0.2750346518820475f),
            new Vector3f(-0.4926118841325975f, -0.3279366743079728f, 0.3683135596740186f),
            new Vector3f(0.2459906458351674f, 0.3647787136629026f, -0.1641662355178652f),
            new Vector3f(0.2100642609503719f, -0.4499717643018549f, 0.3245569875692548f)
        };

        QuickHull3D covexHull = new QuickHull3D(points);

        System.out.println("Vertices:");
        Vector3f[] vertices = covexHull.getFloatVertices();

        Assert.assertEquals(expected.length, vertices.length);
        for (int i = 0; i < vertices.length; i++) {
            Vector3f pnt = vertices[i];
            System.out.println(pnt.x + " " + pnt.y + " " + pnt.z);
            boolean found = false;
            for (Vector3f point3d : expected) {
                double diff = Math.abs(pnt.x - point3d.x) + Math.abs(pnt.y - point3d.y) + Math.abs(pnt.y - point3d.y);
                if (diff < 0.0000001) {
                    found = true;
                }
            }
            Assert.assertTrue(found);
        }

    }
}
