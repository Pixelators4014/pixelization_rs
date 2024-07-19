(function() {var type_impls = {
"nalgebra":[["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Add%3Cusize%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#110-117\">source</a><a href=\"#impl-Add%3Cusize%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Add.html\" title=\"trait core::ops::arith::Add\">Add</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a>&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><details class=\"toggle\" open><summary><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Add.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></summary><div class='docblock'>The resulting type after applying the <code>+</code> operator.</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.add\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#114-116\">source</a><a href=\"#method.add\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Add.html#tymethod.add\" class=\"fn\">add</a>(self, rhs: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a>) -&gt; Self</h4></section></summary><div class='docblock'>Performs the <code>+</code> operation. <a href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Add.html#tymethod.add\">Read more</a></div></details></div></details>","Add<usize>","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Clone-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#impl-Clone-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html\" title=\"trait core::clone::Clone\">Clone</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.clone\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#method.clone\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#tymethod.clone\" class=\"fn\">clone</a>(&amp;self) -&gt; <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></summary><div class='docblock'>Returns a copy of the value. <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#tymethod.clone\">Read more</a></div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.clone_from\" class=\"method trait-impl\"><span class=\"rightside\"><span class=\"since\" title=\"Stable since Rust version 1.0.0\">1.0.0</span> · <a class=\"src\" href=\"https://doc.rust-lang.org/1.79.0/src/core/clone.rs.html#169\">source</a></span><a href=\"#method.clone_from\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#method.clone_from\" class=\"fn\">clone_from</a>(&amp;mut self, source: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.reference.html\">&amp;Self</a>)</h4></section></summary><div class='docblock'>Performs copy-assignment from <code>source</code>. <a href=\"https://doc.rust-lang.org/1.79.0/core/clone/trait.Clone.html#method.clone_from\">Read more</a></div></details></div></details>","Clone","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Debug-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#impl-Debug-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html\" title=\"trait core::fmt::Debug\">Debug</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.fmt\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#method.fmt\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html#tymethod.fmt\" class=\"fn\">fmt</a>(&amp;self, f: &amp;mut <a class=\"struct\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/struct.Formatter.html\" title=\"struct core::fmt::Formatter\">Formatter</a>&lt;'_&gt;) -&gt; <a class=\"type\" href=\"https://doc.rust-lang.org/1.79.0/core/fmt/type.Result.html\" title=\"type core::fmt::Result\">Result</a></h4></section></summary><div class='docblock'>Formats the value using the given formatter. <a href=\"https://doc.rust-lang.org/1.79.0/core/fmt/trait.Debug.html#tymethod.fmt\">Read more</a></div></details></div></details>","Debug","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Dim-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#93-108\">source</a><a href=\"#impl-Dim-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.try_to_usize\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#95-97\">source</a><a href=\"#method.try_to_usize\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.Dim.html#tymethod.try_to_usize\" class=\"fn\">try_to_usize</a>() -&gt; <a class=\"enum\" href=\"https://doc.rust-lang.org/1.79.0/core/option/enum.Option.html\" title=\"enum core::option::Option\">Option</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a>&gt;</h4></section></summary><div class='docblock'>Gets the compile-time value of <code>Self</code>. Returns <code>None</code> if it is not known, i.e., if <code>Self = Dyn</code>.</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.from_usize\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#100-102\">source</a><a href=\"#method.from_usize\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.Dim.html#tymethod.from_usize\" class=\"fn\">from_usize</a>(dim: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a>) -&gt; Self</h4></section></summary><div class='docblock'>Builds an instance of <code>Self</code> from a run-time value. Panics if <code>Self</code> is a type-level\ninteger and <code>dim != Self::try_to_usize().unwrap()</code>.</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.value\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#105-107\">source</a><a href=\"#method.value\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.Dim.html#tymethod.value\" class=\"fn\">value</a>(&amp;self) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a></h4></section></summary><div class='docblock'>Gets the run-time value of <code>self</code>. For type-level integers, this is the same as\n<code>Self::try_to_usize().unwrap()</code>.</div></details><section id=\"method.is\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#76-78\">source</a><a href=\"#method.is\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.Dim.html#method.is\" class=\"fn\">is</a>&lt;D: <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a>&gt;() -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.bool.html\">bool</a></h4></section></div></details>","Dim","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-DimAdd%3CD%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#impl-DimAdd%3CD%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;D: <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a>&gt; <a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimAdd.html\" title=\"trait nalgebra::base::dimension::DimAdd\">DimAdd</a>&lt;D&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"nalgebra/base/dimension/trait.DimAdd.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section><section id=\"method.add\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#method.add\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.DimAdd.html#tymethod.add\" class=\"fn\">add</a>(self, other: D) -&gt; <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></div></details>","DimAdd<D>","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-DimDiv%3CD%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#impl-DimDiv%3CD%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;D: <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a>&gt; <a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimDiv.html\" title=\"trait nalgebra::base::dimension::DimDiv\">DimDiv</a>&lt;D&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"nalgebra/base/dimension/trait.DimDiv.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section><section id=\"method.div\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#method.div\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.DimDiv.html#tymethod.div\" class=\"fn\">div</a>(self, other: D) -&gt; <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></div></details>","DimDiv<D>","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-DimMax%3CD%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#impl-DimMax%3CD%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;D: <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a>&gt; <a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimMax.html\" title=\"trait nalgebra::base::dimension::DimMax\">DimMax</a>&lt;D&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"nalgebra/base/dimension/trait.DimMax.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section><section id=\"method.max\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#method.max\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.DimMax.html#tymethod.max\" class=\"fn\">max</a>(self, other: D) -&gt; <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></div></details>","DimMax<D>","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-DimMin%3CD%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#impl-DimMin%3CD%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;D: <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a>&gt; <a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimMin.html\" title=\"trait nalgebra::base::dimension::DimMin\">DimMin</a>&lt;D&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"nalgebra/base/dimension/trait.DimMin.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section><section id=\"method.min\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#method.min\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.DimMin.html#tymethod.min\" class=\"fn\">min</a>(self, other: D) -&gt; <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></div></details>","DimMin<D>","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-DimMul%3CD%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#impl-DimMul%3CD%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;D: <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a>&gt; <a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimMul.html\" title=\"trait nalgebra::base::dimension::DimMul\">DimMul</a>&lt;D&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"nalgebra/base/dimension/trait.DimMul.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section><section id=\"method.mul\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#method.mul\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.DimMul.html#tymethod.mul\" class=\"fn\">mul</a>(self, other: D) -&gt; <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></div></details>","DimMul<D>","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-DimSub%3CD%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#impl-DimSub%3CD%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl&lt;D: <a class=\"trait\" href=\"nalgebra/base/dimension/trait.Dim.html\" title=\"trait nalgebra::base::dimension::Dim\">Dim</a>&gt; <a class=\"trait\" href=\"nalgebra/base/dimension/trait.DimSub.html\" title=\"trait nalgebra::base::dimension::DimSub\">DimSub</a>&lt;D&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"nalgebra/base/dimension/trait.DimSub.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section><section id=\"method.sub\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#206-213\">source</a><a href=\"#method.sub\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"nalgebra/base/dimension/trait.DimSub.html#tymethod.sub\" class=\"fn\">sub</a>(self, other: D) -&gt; <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></div></details>","DimSub<D>","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#31-38\">source</a><a href=\"#impl-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.new\" class=\"method\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#35-37\">source</a><h4 class=\"code-header\">pub const fn <a href=\"nalgebra/base/dimension/struct.Dyn.html#tymethod.new\" class=\"fn\">new</a>(value: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a>) -&gt; Self</h4></section><span class=\"item-info\"><div class=\"stab deprecated\"><span class=\"emoji\">👎</span><span>Deprecated: use Dyn(value) instead.</span></div></span></summary><div class=\"docblock\"><p>A dynamic size equal to <code>value</code>.</p>\n</div></details></div></details>",0,"nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-PartialEq-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#impl-PartialEq-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html\" title=\"trait core::cmp::PartialEq\">PartialEq</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><details class=\"toggle method-toggle\" open><summary><section id=\"method.eq\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#method.eq\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html#tymethod.eq\" class=\"fn\">eq</a>(&amp;self, other: &amp;<a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a>) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.bool.html\">bool</a></h4></section></summary><div class='docblock'>This method tests for <code>self</code> and <code>other</code> values to be equal, and is used\nby <code>==</code>.</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.ne\" class=\"method trait-impl\"><span class=\"rightside\"><span class=\"since\" title=\"Stable since Rust version 1.0.0\">1.0.0</span> · <a class=\"src\" href=\"https://doc.rust-lang.org/1.79.0/src/core/cmp.rs.html#263\">source</a></span><a href=\"#method.ne\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.PartialEq.html#method.ne\" class=\"fn\">ne</a>(&amp;self, other: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.reference.html\">&amp;Rhs</a>) -&gt; <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.bool.html\">bool</a></h4></section></summary><div class='docblock'>This method tests for <code>!=</code>. The default implementation is almost always\nsufficient, and should not be overridden without very good reason.</div></details></div></details>","PartialEq","nalgebra::base::dimension::Dynamic"],["<details class=\"toggle implementors-toggle\" open><summary><section id=\"impl-Sub%3Cusize%3E-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#119-126\">source</a><a href=\"#impl-Sub%3Cusize%3E-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Sub.html\" title=\"trait core::ops::arith::Sub\">Sub</a>&lt;<a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a>&gt; for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section></summary><div class=\"impl-items\"><details class=\"toggle\" open><summary><section id=\"associatedtype.Output\" class=\"associatedtype trait-impl\"><a href=\"#associatedtype.Output\" class=\"anchor\">§</a><h4 class=\"code-header\">type <a href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Sub.html#associatedtype.Output\" class=\"associatedtype\">Output</a> = <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h4></section></summary><div class='docblock'>The resulting type after applying the <code>-</code> operator.</div></details><details class=\"toggle method-toggle\" open><summary><section id=\"method.sub\" class=\"method trait-impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#123-125\">source</a><a href=\"#method.sub\" class=\"anchor\">§</a><h4 class=\"code-header\">fn <a href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Sub.html#tymethod.sub\" class=\"fn\">sub</a>(self, rhs: <a class=\"primitive\" href=\"https://doc.rust-lang.org/1.79.0/std/primitive.usize.html\">usize</a>) -&gt; Self</h4></section></summary><div class='docblock'>Performs the <code>-</code> operation. <a href=\"https://doc.rust-lang.org/1.79.0/core/ops/arith/trait.Sub.html#tymethod.sub\">Read more</a></div></details></div></details>","Sub<usize>","nalgebra::base::dimension::Dynamic"],["<section id=\"impl-Copy-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#impl-Copy-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/marker/trait.Copy.html\" title=\"trait core::marker::Copy\">Copy</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section>","Copy","nalgebra::base::dimension::Dynamic"],["<section id=\"impl-Eq-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#impl-Eq-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/cmp/trait.Eq.html\" title=\"trait core::cmp::Eq\">Eq</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section>","Eq","nalgebra::base::dimension::Dynamic"],["<section id=\"impl-IsDynamic-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#65\">source</a><a href=\"#impl-IsDynamic-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"nalgebra/base/dimension/trait.IsDynamic.html\" title=\"trait nalgebra::base::dimension::IsDynamic\">IsDynamic</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section>","IsDynamic","nalgebra::base::dimension::Dynamic"],["<section id=\"impl-IsNotStaticOne-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#66\">source</a><a href=\"#impl-IsNotStaticOne-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"nalgebra/base/dimension/trait.IsNotStaticOne.html\" title=\"trait nalgebra::base::dimension::IsNotStaticOne\">IsNotStaticOne</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section>","IsNotStaticOne","nalgebra::base::dimension::Dynamic"],["<section id=\"impl-StructuralPartialEq-for-Dyn\" class=\"impl\"><a class=\"src rightside\" href=\"src/nalgebra/base/dimension.rs.html#17\">source</a><a href=\"#impl-StructuralPartialEq-for-Dyn\" class=\"anchor\">§</a><h3 class=\"code-header\">impl <a class=\"trait\" href=\"https://doc.rust-lang.org/1.79.0/core/marker/trait.StructuralPartialEq.html\" title=\"trait core::marker::StructuralPartialEq\">StructuralPartialEq</a> for <a class=\"struct\" href=\"nalgebra/base/dimension/struct.Dyn.html\" title=\"struct nalgebra::base::dimension::Dyn\">Dyn</a></h3></section>","StructuralPartialEq","nalgebra::base::dimension::Dynamic"]]
};if (window.register_type_impls) {window.register_type_impls(type_impls);} else {window.pending_type_impls = type_impls;}})()